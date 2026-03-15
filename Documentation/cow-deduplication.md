# CRIU COW Page Deduplication Design Document

## Executive Summary

This document describes a design for implementing Copy-On-Write (COW) page
deduplication in CRIU. The optimization targets fork-intensive workloads by
avoiding duplicate storage of shared pages between parent and child processes.

**Expected Benefits:**

- **Storage reduction:** 30-50% for fork-intensive applications
- **Restore speedup:** ~100x faster for COW page restoration
- **Dump overhead:** 2-3x slower (acceptable trade-off)

---

## Problem Statement

### Current Behavior

CRIU currently handles parent-child process relationships inefficiently during
checkpoint:

**Dump Phase:**

- Each process is dumped independently in DFS order (parent before children)
- Pages shared via COW after `fork()` are stored separately for each process
- No detection of physically shared pages between parent and child

**Restore Phase:**

- CRIU detects potential COW pages by matching VMAs between parent and child
- Uses `mremap()` to share parent's physical pages with child
- Compares every candidate page via `memcmp()` at runtime (expensive)
- Only then decides whether to keep the shared page or overwrite it

**Example Redundancy:**

```
Process Tree:
  Parent A: 100MB memory
  Child B (forked from A): 90% pages unchanged

Current Storage:
  Parent A: 100MB
  Child B:  100MB (90MB duplicated)
  Total:    200MB

Restore Cost:
  - Read 100MB for child
  - memcmp() 90MB worth of pages
  - Time: ~2.5s of unnecessary work
```

---

## Solution Overview

### Core Idea

**Leverage dump-time information to optimize restore:**

1. **Dump Phase:** Detect COW pages using content comparison
2. **Storage:** Mark COW pages in pagemap; optionally hole-punch page data
3. **Restore Phase:** Trust dump-time information, skip runtime detection

### Key Insight

CRIU's current restore mechanism already establishes memory sharing via
`mremap()` in `premap_private_vma()`. We only need to signal which pages to
skip during the read-and-compare step in `restore_priv_vma_content()`.

---

## Detailed Design

### 1. Data Structure Extensions

#### Protobuf Schema Update

**File:** `images/pagemap.proto`

The current schema (all 5 existing fields shown) with the new addition:

```protobuf
message pagemap_entry {
    required uint64 vaddr           = 1 [(criu).hex = true];
    required uint32 compat_nr_pages = 2;          // existing
    optional bool   in_parent       = 3;          // existing: predump parent snapshot
    optional uint32 flags           = 4 [(criu).flags = "pmap.flags"]; // existing
    optional uint64 nr_pages        = 5;          // existing
    optional uint32 parent_pid      = 6;          // NEW: live parent PID for COW ref
}
```

> **Important:** `in_parent` (field 3) is an unrelated, existing boolean used
> by the pre-dump snapshot chain. The new `parent_pid` field (field 6)
> identifies the live parent process from which a page was confirmed identical
> at dump time. These two fields serve different mechanisms and must not be
> confused.

#### Flag Definitions

**File:** `criu/include/pagemap.h`

```c
/* Existing flags -- do not change values */
#define PE_PARENT  (1 << 0) /* pages are in parent snapshot */
#define PE_LAZY    (1 << 1) /* pages can be lazily restored */
#define PE_PRESENT (1 << 2) /* pages are present in pages*img */

/* New flag */
#define PE_PARENT_PROC (1 << 3) /* pages confirmed COW with live parent at dump */

static inline bool pagemap_in_parent_proc(PagemapEntry *pe)
{
	return !!(pe->flags & PE_PARENT_PROC);
}
```

#### Dump-Time VMA Cache in `dmp_info`

**File:** `criu/include/pstree.h`

`dmp_info` is the existing dump-time counterpart to `rst_info`, accessed via
`dmpi(item)`. We extend it with one new field:

```c
struct dmp_info {
	struct ns_id		    *netns;
	struct page_pipe	    *mem_pp;
	struct parasite_ctl	    *parasite_ctl;
	struct parasite_thread_ctl **thread_ctls;
	uint64_t		    *thread_sp;
	struct criu_rseq_cs	    *thread_rseq_cs;
	struct thread_lsm	    **thread_lsms;

	/*
	 * NEW: Pointer to the collected VMA list for this item.
	 * Set after VMA collection during dump so child processes
	 * can look up parent VMAs via dmpi(item->parent)->vma_area_list.
	 */
	struct vm_area_list	    *vma_area_list;
};
```

**In the dump flow**, after calling `collect_mappings()` for each item, store
the pointer:

```c
dmpi(item)->vma_area_list = vma_area_list;
```

Children are dumped after their parent in DFS order, so
`dmpi(item->parent)->vma_area_list` is always valid when a child is being
dumped.

---

### 2. Dump Phase Implementation

#### Overview

```
For each child process (in DFS order, after parent is dumped):
  1. Retrieve parent's vm_area_list via dmpi(item->parent)->vma_area_list
  2. Iterate through each VMA of the child
  3. Find matching VMA in parent's list
  4. For each page:
     a) Read parent's page content via process_vm_readv()
     b) Read child's page content via process_vm_readv()
     c) Compare with memcmp()
     d) If identical:
        - Mark pagemap entry with PE_PARENT_PROC
        - Record parent_pid in pagemap entry
        - Skip writing page data (or hole-punch in aggressive mode)
     e) If different:
        - Mark pagemap entry with PE_PRESENT
        - Write page data normally
```

#### Core Detection Function

**File:** `criu/mem.c`

```c
#include <sys/uio.h>

/*
 * Read a single page from a process using process_vm_readv().
 * Returns 0 on success, -EFAULT if page is not present, -1 on error.
 */
static int read_process_page(pid_t pid, unsigned long vaddr, void *buf)
{
	struct iovec local = {
		.iov_base = buf,
		.iov_len  = PAGE_SIZE
	};
	struct iovec remote = {
		.iov_base = (void *)vaddr,
		.iov_len  = PAGE_SIZE
	};
	ssize_t ret;

	ret = process_vm_readv(pid, &local, 1, &remote, 1, 0);
	if (ret < 0) {
		if (errno == EFAULT)
			return -EFAULT;
		pr_perror("process_vm_readv failed for pid %d addr %lx",
			  pid, vaddr);
		return -1;
	}

	if (ret != PAGE_SIZE) {
		pr_err("Short read: %zd bytes for pid %d addr %lx\n",
		       ret, pid, vaddr);
		return -1;
	}

	return 0;
}

/*
 * Check if a page is COW-identical between parent and child by comparing
 * content. All processes are frozen at this point so there is no race.
 *
 * Returns true if pages have identical content and can be treated as COW.
 */
static bool is_cow_page_by_content(pid_t ppid, pid_t cpid,
				    unsigned long vaddr)
{
	unsigned char parent_page[PAGE_SIZE];
	unsigned char child_page[PAGE_SIZE];
	int ret;

	ret = read_process_page(ppid, vaddr, parent_page);
	if (ret < 0)
		return false;

	ret = read_process_page(cpid, vaddr, child_page);
	if (ret < 0)
		return false;

	return memcmp(parent_page, child_page, PAGE_SIZE) == 0;
}
```

#### VMA Matching Logic

```c
/*
 * Find the VMA in the parent's collected vm_area_list that corresponds
 * to child_vma. Uses dmpi(parent)->vma_area_list, which is set during
 * the parent's dump pass before any child is processed.
 *
 * Returns parent vma_area if found, NULL otherwise.
 */
static struct vma_area *find_parent_vma(struct pstree_item *parent,
					struct vma_area *child_vma)
{
	struct vm_area_list *pvmas;
	struct vma_area *pvma;

	if (!parent || !task_alive(parent))
		return NULL;

	pvmas = dmpi(parent)->vma_area_list;
	if (!pvmas)
		return NULL;

	list_for_each_entry(pvma, &pvmas->h, list) {
		if (pvma->e->start != child_vma->e->start)
			continue;
		if (pvma->e->end != child_vma->e->end)
			continue;

		/* Both must be private mappings */
		if (!vma_area_is_private(pvma, kdat.task_size))
			continue;
		if (!vma_area_is_private(child_vma, kdat.task_size))
			continue;

		/* Hugetlb pages behave differently, skip */
		if (pvma->e->flags & MAP_HUGETLB)
			continue;

		/* MAP_GROWSDOWN and MAP_ANONYMOUS must match */
		if ((pvma->e->flags ^ child_vma->e->flags) &
		    (MAP_GROWSDOWN | MAP_ANONYMOUS))
			continue;

		/* For file-backed mappings, must be same file */
		if (!(pvma->e->flags & MAP_ANONYMOUS) &&
		    pvma->e->shmid != child_vma->e->shmid)
			continue;

		return pvma;
	}

	return NULL;
}
```

#### Main Dump Integration

This function is called from within `__parasite_dump_pages_seized()` as a
replacement for the normal per-page dump loop when a parent exists and COW
dedup is enabled:

```c
/*
 * Dump pages with COW deduplication.
 * Accesses parent VMAs via dmpi(item->parent)->vma_area_list, which
 * is guaranteed to be set because parents are always dumped before
 * children in DFS order.
 */
static int dump_pages_with_cow_dedup(struct pstree_item *item,
				      struct vm_area_list *vma_area_list,
				      struct page_xfer *xfer)
{
	struct pstree_item *parent = item->parent;
	struct vma_area *vma;
	pid_t cpid, ppid = 0;
	unsigned long cow_pages = 0, total_pages = 0;
	bool has_parent;
	int ret = 0;

	cpid = vpid(item);
	has_parent = (parent != NULL && task_alive(parent));
	if (has_parent)
		ppid = vpid(parent);

	pr_info("Dumping pid %d COW dedup %s\n",
		cpid, has_parent ? "enabled" : "disabled");

	list_for_each_entry(vma, &vma_area_list->h, list) {
		struct vma_area *pvma = NULL;
		unsigned long addr;

		if (!vma_area_is_private(vma, kdat.task_size))
			continue;
		if (vma_area_is(vma, VMA_AREA_VSYSCALL) ||
		    vma_area_is(vma, VMA_AREA_VDSO))
			continue;

		if (has_parent)
			pvma = find_parent_vma(parent, vma);

		for (addr = vma->e->start; addr < vma->e->end;
		     addr += PAGE_SIZE) {
			struct iovec iov = {
				.iov_base = (void *)addr,
				.iov_len  = PAGE_SIZE
			};
			bool is_cow = false;

			total_pages++;

			if (pvma)
				is_cow = is_cow_page_by_content(ppid, cpid,
								addr);

			if (is_cow) {
				if (xfer->write_pagemap(xfer, &iov,
							PE_PARENT_PROC) < 0) {
					ret = -1;
					goto out;
				}
				cow_pages++;
				cnt_add(CNT_PAGES_DUMP_COW, 1);
			} else {
				ret = dump_page_normal(ctl, addr, xfer);
				if (ret < 0)
					goto out;
			}

			cnt_add(CNT_PAGES_DUMP_COW_SCANNED, 1);
		}
	}

	pr_info("COW dedup: %lu/%lu pages deduplicated (%.1f%%)\n",
		cow_pages, total_pages,
		total_pages ? (100.0 * cow_pages / total_pages) : 0.0);
out:
	return ret;
}
```

---

### 3. Restore Phase Implementation

The restore mechanism leverages existing COW infrastructure with a targeted
change to `restore_priv_vma_content()` in `criu/mem.c`.

#### What Already Exists (Unchanged)

```
1. prepare_cow_vmas()    -- Match parent/child VMAs by address range
2. premap_private_vma()  -- Use mremap() to share parent's physical pages
3. restore_priv_vma_content() -- Restore page contents (modified below)
```

The existing code at `mem.c:1219-1232` reads every inherited page from the
image and does a `memcmp()` to decide whether to overwrite the shared page.
The optimization skips this read+compare entirely for pages marked
`PE_PARENT_PROC`.

#### Modified Section of `restore_priv_vma_content()`

Only the `vma_inherited` branch changes. The surrounding function signature
and structure remain identical to `mem.c:1119`:

```c
static int restore_priv_vma_content(struct pstree_item *t,
				     struct page_read *pr)
{
	/* ... existing local variable declarations unchanged ... */
	unsigned int nr_cow_flagged = 0; /* NEW: pages skipped via PE_PARENT_PROC */

	/* ... existing setup unchanged ... */

	while (1) {
		/* ... existing advance/lazy logic unchanged ... */

		for (i = 0; i < nr_pages; i++) {
			unsigned char buf[PAGE_SIZE];
			void *p;

			/* ... existing VMA lookup and non-premapped path unchanged ... */

			/*
			 * COW path for premapped VMAs
			 * (existing location: mem.c:1215)
			 */
			off = (va - vma->e->start) / PAGE_SIZE;
			p = decode_pointer((off) * PAGE_SIZE +
					   vma->premmaped_addr);

			set_bit(off, vma->page_bitmap);
			if (vma_inherited(vma)) {
				clear_bit(off, vma->pvma->page_bitmap);

				/*
				 * NEW: If this page was confirmed COW at dump
				 * time, mremap() has already shared the
				 * parent's physical page. Skip the read and
				 * compare entirely.
				 */
				if (pagemap_in_parent_proc(pr->pe)) {
					nr_shared++;
					nr_cow_flagged++;
					va += PAGE_SIZE;
					continue;
				}

				/*
				 * Existing path: page was not flagged at dump
				 * time (old image or modified page). Fall back
				 * to runtime content comparison.
				 */
				ret = pr->read_pages(pr, va, 1, buf, 0);
				if (ret < 0)
					goto err_read;

				va += PAGE_SIZE;
				nr_compared++;
				cnt_add(CNT_PAGES_COMPARED, 1);

				if (memcmp(p, buf, PAGE_SIZE) == 0) {
					nr_shared++;
					cnt_add(CNT_PAGES_SKIPPED_COW, 1);
					continue;
				}

				nr_restored++;
				memcpy(p, buf, PAGE_SIZE);
			} else {
				/* ... existing non-inherited path unchanged ... */
			}
		}
	}

	/* ... existing err_read / sync / close / MADV_DONTNEED cleanup unchanged ... */

	pr_info("Page restore summary for pid %d:\n", vpid(t));
	pr_info("  nr_restored:    %u\n", nr_restored);
	pr_info("  nr_shared:      %u\n", nr_shared);
	pr_info("  nr_cow_flagged: %u (skipped via PE_PARENT_PROC)\n",
		nr_cow_flagged);
	pr_info("  nr_compared:    %u (runtime fallback)\n", nr_compared);
	return 0;

err_addr:
	pr_err("Page entry address %lx outside VMA\n", va);
	return -1;
}
```

---

### 4. Statistics Counters

**File:** `criu/include/stats.h`

The existing counters are left untouched. New dump-time counters are added
under the dump enum:

```c
enum {
	CNT_PAGES_SCANNED,
	CNT_PAGES_SKIPPED_PARENT,
	CNT_PAGES_WRITTEN,
	CNT_PAGES_LAZY,
	CNT_PAGE_PIPES,
	CNT_PAGE_PIPE_BUFS,
	CNT_SHPAGES_SCANNED,
	CNT_SHPAGES_SKIPPED_PARENT,
	CNT_SHPAGES_WRITTEN,

	/* NEW: COW dedup dump-time counters */
	CNT_PAGES_DUMP_COW_SCANNED, /* pages examined for COW identity */
	CNT_PAGES_DUMP_COW,         /* pages confirmed COW, not written */

	DUMP_CNT_NR_STATS,
};

/* Existing restore counters -- reused, not renamed */
enum {
	CNT_PAGES_COMPARED,    /* pages compared at restore via memcmp() */
	CNT_PAGES_SKIPPED_COW, /* pages skipped by runtime COW detection */
	CNT_PAGES_RESTORED,

	RESTORE_CNT_NR_STATS,
};
```

> `CNT_PAGES_COMPARED` and `CNT_PAGES_SKIPPED_COW` already exist and continue
> to count runtime-detected COW pages. The `nr_cow_flagged` local variable is
> logged per-process but not promoted to a global counter to avoid bloating the
> stats image; it can be added in a follow-up if needed.

---

## Performance Analysis

### Dump Phase

```
Per-page overhead:
  - 2x process_vm_readv() (parent + child): ~200us
  - memcmp(4KB):                             ~10us
  Total:                                    ~210us/page

For 100MB process (25,600 pages):
  Time = 25,600 x 210us = 5.38s  (vs ~2s current)
  Overhead: ~2.7x slower
```

### Restore Phase

```
Current (runtime detection, mem.c:1222-1235):
  Per COW page: read_pages() + memcmp() ~= 110us

Optimized (PE_PARENT_PROC flagged):
  Per COW page: flag check ~= 1ns

Speedup: ~110,000x

For 100MB child, 90% COW (23,040 pages):
  Current: 23,040 x 110us = 2.53s
  New:     23,040 x 1ns   = 0.023ms
```

### Storage Savings

```
Parent 100MB + child forked (90% unchanged):

Current:          200MB
With COW dedup:   ~110MB  (45% reduction)
```

### Overall Impact

```
Fork-intensive workload (1 parent + 5 children, 90% COW):

Dump:
  Current: 6 x 2s     = 12s
  New:     6 x 5.38s  = 32.3s
  Cost:    +20.3s (one-time)

Restore (performed multiple times):
  Current: 6 x 2.53s = 15.2s
  New:     6 x 0.02s = 0.12s
  Savings: 15.1s per restore

Break-even: after 2 restores, total time saved exceeds dump overhead
```

---

## Edge Cases & Considerations

### 1. Multiple Children

Each child is compared only with its immediate parent via
`dmpi(item->parent)->vma_area_list`. Sibling deduplication is a future
extension.

### 2. Exec After Fork

`find_parent_vma()` will find no matching VMAs (different `shmid`, sizes, or
mapping flags). Falls back to normal dump automatically.

### 3. Race Conditions

All processes are frozen before dump starts (via ptrace/freezer cgroup).
Memory is stable for the entire duration. No race is possible.

### 4. Huge Pages

`MAP_HUGETLB` VMAs are excluded in `find_parent_vma()`. They fall back to
normal dump.

### 5. File-Backed Private Mappings

Supported: the `shmid` check in `find_parent_vma()` ensures both sides map
the same file. Common case: shared library `.data` sections.

### 6. Zombie Processes

The `task_alive()` check at the top of `find_parent_vma()` prevents any
attempt to compare a zombie's pages.

---

## Backward Compatibility

### New CRIU Restoring Old Images

`pagemap_in_parent_proc()` returns false (flag not set). Falls back to the
existing `read_pages()` + `memcmp()` path. Fully compatible.

### Old CRIU Restoring New Images

- Protobuf `optional` fields (`parent_pid = 6`) are silently ignored by old
  parsers.
- Old CRIU does not understand `PE_PARENT_PROC` in `flags` (field 4). It will
  attempt to read the page from `pages.img`.
- **If hole punching is NOT used** (default conservative mode): page data is
  still present in `pages.img`; old CRIU reads and compares it as usual.
  Works correctly.
- **If hole punching IS used** (opt-in `--cow-dedup-aggressive`): page data is
  absent; old CRIU will fail. This mode is explicitly documented as
  incompatible with older CRIU versions.

### Recommendation

**Default:** write page data normally even for COW pages (pagemap entry gets
`PE_PARENT_PROC`, page data is still stored). This gives the restore speedup
immediately with zero compatibility risk.

**Opt-in:** `--cow-dedup-aggressive` enables hole punching for storage
savings, with documented incompatibility with older CRIU versions.

---

## Implementation Phases

### Phase 1: Core Infrastructure (Week 1-2)

1. Add `vma_area_list` field to `dmp_info` (`criu/include/pstree.h`)
2. Set `dmpi(item)->vma_area_list` after VMA collection in dump flow
3. Add `parent_pid = 6` to `images/pagemap.proto`
4. Add `PE_PARENT_PROC` flag and `pagemap_in_parent_proc()` to
   `criu/include/pagemap.h`
5. Add `CNT_PAGES_DUMP_COW_SCANNED` and `CNT_PAGES_DUMP_COW` to
   `criu/include/stats.h`

### Phase 2: Dump Integration (Week 3-4)

1. Implement `read_process_page()` and `is_cow_page_by_content()` in
   `criu/mem.c`
2. Implement `find_parent_vma()` using `dmpi(parent)->vma_area_list`
3. Implement `dump_pages_with_cow_dedup()` and integrate into
   `__parasite_dump_pages_seized()`
4. Test with simple fork scenarios

### Phase 3: Restore Integration (Week 5-6)

1. Add `PE_PARENT_PROC` fast-path in `restore_priv_vma_content()`
   (`criu/mem.c:1219`)
2. Verify backward compatibility with old images
3. Add performance counters and logging

### Phase 4: Testing & Optimization (Week 7-8)

1. ZDTM test suite additions (see Testing section)
2. Performance benchmarking
3. Implement `--cow-dedup-aggressive` (hole punching) as opt-in
4. Documentation updates

---

## Testing Strategy

### ZDTM Tests

**`test/zdtm/static/cow_simple.c`**
- Fork child with no modifications, checkpoint both
- Verify child's `pages.img` is small; restore produces correct memory

**`test/zdtm/static/cow_partial.c`**
- Fork child, modify 10% of pages
- Verify only modified pages appear in child's `pages.img`

**`test/zdtm/static/cow_chain.c`**
- Parent -> Child -> Grandchild chain
- Verify multi-level dedup works correctly

**`test/zdtm/static/cow_exec.c`**
- Fork then exec
- Verify no COW dedup attempted (different mappings)

### Backward Compatibility Tests

**`test/backward_compat/dump_old_restore_new.sh`**
- Dump without `--cow-dedup`, restore with new CRIU

**`test/backward_compat/dump_new_restore_old.sh`**
- Dump with `--cow-dedup` (non-aggressive), restore with old CRIU

---

## Alternative Approaches Considered

### PFN-Based Detection

Reading Page Frame Numbers from `/proc/pid/pagemap` to directly detect shared
physical pages.

- **Pro:** ~100x faster than content comparison
- **Con:** PFN field requires `CAP_SYS_ADMIN` (zeroed since Linux 4.2 for
  unprivileged users, due to Rowhammer mitigations)
- **Decision:** Not used as primary method. Can be added as a fast path when
  available:

```c
if (can_access_pfn())
	is_cow = is_cow_page_pfn(ppid, cpid, addr);
else
	is_cow = is_cow_page_by_content(ppid, cpid, addr);
```

### Soft-Dirty Bit Filtering

Use soft-dirty tracking to pre-filter candidate pages before content
comparison.

- **Pro:** Reduces number of `memcmp()` calls
- **Con:** Requires extra kernel state setup; still needs content verification
- **Decision:** Deferred as a future optimization

### Parasite-Based Detection

Inject parasite code to compare pages in-process.

- **Con:** Complex IPC, parasite injection overhead, more fragile
- **Decision:** Rejected; `process_vm_readv()` is simpler and already used
  elsewhere in CRIU

---

## Security Considerations

- `process_vm_readv()` requires `CAP_SYS_PTRACE`, which CRIU already holds.
  No new privileges required.
- All comparisons are performed within a single checkpoint session on frozen,
  already-seized processes. No new information exposure.
- 2-3x dump slowdown is bounded and opt-in. No denial-of-service risk beyond
  what dump already incurs.

---

## Future Enhancements

1. **PFN fast path:** Use PFN comparison when `CAP_SYS_ADMIN` is available,
   fall back to content comparison otherwise
2. **Cross-sibling deduplication:** Hash pages with SHA-256 and deduplicate
   across siblings in a post-processing pass
3. **Soft-dirty pre-filter:** Skip content comparison for pages that
   soft-dirty tracking shows were modified
4. **Incremental COW tracking:** Across pre-dump iterations, track which pages
   have been COW-broken to avoid re-comparing them

---

## Conclusion

COW page deduplication targets fork-intensive workloads with storage and
restore-time benefits. The design integrates cleanly into existing CRIU
infrastructure:

- **Dump side:** `dmpi(item->parent)->vma_area_list` gives access to the
  parent's VMAs without introducing a new VMA lookup mechanism
- **Restore side:** A single flag check in the existing `vma_inherited` branch
  of `restore_priv_vma_content()` eliminates the `read_pages()` + `memcmp()`
  cost for flagged pages
- **Compatibility:** Conservative default (no hole punching) is fully
  bi-directionally compatible with old images and old CRIU

**Key Success Metrics:**

- 30-50% storage reduction for fork workloads
- ~100x faster restore for COW pages
- Acceptable 2-3x dump slowdown (opt-in)
- Full backward compatibility by default
- No additional privilege requirements

---

**Document Version:** 1.1 (errors corrected)
**Last Updated:** 2026-03-15
