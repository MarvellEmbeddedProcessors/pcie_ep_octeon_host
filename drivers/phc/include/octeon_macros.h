/* Copyright (c) 2020 Marvell.
 * SPDX-License-Identifier: GPL-2.0
 */

/*! \file octeon_macros.h 
    \brief Host driver: Inline macros for memory allocation/free and dispatch
           list handling. 
 */

#ifndef __OCTEON_MACROS_H__
#define __OCTEON_MACROS_H__

#define     PTR_TO_ULL(x)             ((unsigned long long)(unsigned long)(x))
#define     CVM_MIN(d1, d2)           (((d1) < (d2)) ? (d1) : (d2))
#define     CVM_MAX(d1, d2)           (((d1) > (d2))?(d1):(d2))

#define     CVM_CHECK_BIT(var, pos)   ((var) & ((1UL) <<(pos)))
#define     CVM_SET_BIT(var, pos)     ((var) |= ((1UL) << (pos)))
#define     CVM_CLEAR_BIT(var, pos)   ((var) &= ( ~(1UL) << (pos)))

/*
 *  Macros that switch between using the buffer pool and the system-dependent
 *  routines based on compilation flags used.
 */
#define cavium_alloc_buffer(octeon_dev, size)   cavium_malloc_dma(size, __CAVIUM_MEM_ATOMIC)
#define cavium_free_buffer(octeon_dev, buf)     cavium_free_dma(buf)

/** Allocates with page size granularity.
 *  @param size        - size of memory to allocate.
 *  @param alloc_size  - pointer to 32-bit location where actual allocated
 *                       size is returned.
 *  @param orig_ptr    - If the ptr was moved to make the memory buffer
 *                       8-byte aligned, the start address of allocated
 *                       memory is returned here.
 *  @param ctx         - currently unsed
 *  @return If allocation is successful, the start of the 8-byte aligned
 *          buffer is returned, else NULL.
 */
static inline void *cavium_alloc_aligned_memory(uint32_t size,
						uint32_t * alloc_size,
						unsigned long *orig_ptr,
						void *ctx)
{
	return cnnic_alloc_aligned_dma(size, alloc_size, orig_ptr, ctx);
}


static inline int
cvm_is_val_in_range(uint32_t index, uint32_t minVal, uint32_t maxVal)
{
	if (minVal < maxVal)
		return (index >= minVal) && (index < maxVal);
	else
		return (index < maxVal) || (index >= minVal);
}

/*
  off   - offset in dst where data will be copied.
  len   - bytes to be copied
  size  - size of dst buffer
  If (off+len) <= size, copy len bytes from src to (dst + off).
  Return the new offset
 */
static inline int
cvm_copy_cond(char *dst, char *src, int off, int len, int size)
{
	if ((off + len) <= size) {
		cavium_memcpy((dst + off), src, len);
		off += len;
	}

	return off;
}

#endif

/* $Id: octeon_macros.h 141410 2016-06-30 14:37:41Z mchalla $ */
