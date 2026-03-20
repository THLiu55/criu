## COW Memory Deduplication

### 1. Goal

Reduce CRIU checkpoint image size by detecting at **dump time** which pages in a
child process are still kernel Copy-on-Write (COW) shared with the parent, and
skipping writing those pages to disk. At restore time the page is already correct
in the child's address space via CRIU's existing mremap-based COW inheritance
mechanism.

### 2. Background: Existing Restore-Side COW Inheritance

Before this work, CRIU already had a restore-time optimization:

1. `prepare_cow_vmas()` walks each task's VMA list and, for VMAs that exist in
   both parent and child at the same address range with matching flags, sets
   `vma->pvma` to point to the parent's VMA (`vma_inherited(vma)` returns true).
2. During memory restore, inherited VMAs are mapped via `mremap()` from the
   parent's already-restored pages, giving the child a COW reference for free.
3. When a child page is read from the image and the parent's copy already matches,
   `pages_skipped_cow` is incremented and the parent's page is reused.

The baseline measurement showed **~80–82 pages skipped COW** at restore for the
`zdtm/static/cow00` test. The goal was to match this at dump time — skip writing
those same pages to `pages.img`.

### 3. Implementation

**3.1 New Option: `--cow-dedup`**

- `criu/include/cr_options.h`: added `bool cow_dedup` to `struct cr_options`
- `criu/config.c`: registered `BOOL_OPT("cow-dedup", &opts.cow_dedup)`
- `criu/crtools.c`: added help text
- `criu/cr-service.c` + `images/rpc.proto`: RPC support

**3.2 New Pagemap Flag: `PE_PARENT_PROC`**

Added `PE_PARENT_PROC (1 << 3)` to `criu/include/pagemap.h` and
`images/pagemap.proto`. When set on a pagemap entry, the corresponding page was
not written to `pages.img`; at restore the child uses the parent's
already-restored page.

**3.3 New Page-Pipe Hole Type: `PP_HOLE_COW_PARENT`**

Added `PP_HOLE_COW_PARENT (1 << 1)` to `criu/include/page-pipe.h`. When
`generate_iovs()` decides a page can be deduplicated, instead of adding it to the
data pipe it inserts a COW hole. `page-xfer.c`'s `get_hole_flags()` maps this
hole type to the `PE_PARENT_PROC` pagemap flag when writing to the image.

**3.4 Saving Parent VMAs at Dump Time**

In `criu/cr-dump.c`, after collecting a task's VMA list, if `opts.cow_dedup` is
set and the task has children, the VMA list is saved into shared memory and stored
in `dmpi(item)->vma_area_list` (new field in `struct dmp_info` in
`criu/include/pstree.h`). Child processes look up this list when deciding which
of their pages to deduplicate.

**3.5 Dump-Time VMA Matching: `build_dump_cow_pvma_map()`**

At dump time, for each child task, a single pre-pass over the child and
parent VMA lists is performed before the page loop. This mirrors the exact
merge-sort scan algorithm used by `prepare_cow_vmas_for()` at restore time,
ensuring the two phases agree on which VMAs are inheritable:

- Advance the child cursor while `child->start <= parent->start` (the `<=` means
  a child VMA at the same address as the current parent is skipped, not matched —
  identical to the restore-time behavior).
- Advance the parent cursor while `parent->start < child->start`.
- On an address match, validate with `check_cow_vmas()` (same conditions as
  restore time: equal end address, both private, no `MAP_HUGETLB`, matching
  `MAP_GROWSDOWN`/`MAP_ANONYMOUS` flags, same `shmid` for file-backed VMAs).

Results are stored in a heap-allocated array (`struct vma_area **`) indexed by
VMA list position. The per-VMA page loop does an O(1) lookup — overall complexity
is O(N+M) per task, the same as the restore-time pass.

**3.6 COW Detection Per Page**

For each private page in a VMA that has a matched parent VMA:

* Phase 1 — PFN comparison (`is_cow_page_by_pfn`):
  Read `/proc/ppid/pagemap` and `/proc/cpid/pagemap` for the virtual address.
  If the Page Frame Numbers match, the kernel still shares the same physical page.
  Mark with `PP_HOLE_COW_PARENT`.

* Phase 2 — Content comparison (`is_cow_page_by_content`):
  If PFNs differ (child broke COW but wrote back identical content), use
  `process_vm_readv()` + `memcmp()`. Both processes are frozen, so identical
  content means the child can safely use the parent's restored page.
  Mark with `PP_HOLE_COW_PARENT`.

**3.7 Restore Side: `restore_priv_vma_content()`**

When a pagemap entry has `PE_PARENT_PROC` and the VMA is inherited
(`vma_inherited(vma)` is true), call `pr->skip_pages(pr, PAGE_SIZE)` to advance
the page-reader without consuming image data. The parent's already-restored page
is already mapped in the child's address space via mremap.

The design invariant: `build_dump_cow_pvma_map()` uses the identical scan
algorithm as `prepare_cow_vmas_for()`, so every page marked `PE_PARENT_PROC` at
dump time is guaranteed to have `vma->pvma != NULL` at restore time. 

### 4. Test Results

* Test: `zdtm/static/cow00` with `--cow-dedup --display-stats`

  ![](/Users/liutianhao/Documents/Xnip2026-03-20_09-54-30.jpg)

  The dump-time dedup count matches the restore-time skipped-COW count, confirming the two phases are consistent.

* All zdtm tests passed.

### 5. Future Works

What I’m doing now is essentially moving CRIU’s existing parent-child COW shared-page reuse mechanism from the restore phase to the dump phase, so that during checkpointing we can avoid writing many pages that do not actually need to be stored. To ensure correctness first, I have not focused too much on optimizing the dump-time COW detection versus the restore-time duplicate-page detection logic. Further work can then focus on improving performance.