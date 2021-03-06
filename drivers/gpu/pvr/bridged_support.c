/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#include "img_defs.h"
#include "servicesint.h"
#include "bridged_support.h"

enum PVRSRV_ERROR
PVRSRVLookupOSMemHandle(struct PVRSRV_HANDLE_BASE *psHandleBase,
			void **phOSMemHandle, void *hMHandle)
{
	void *hMHandleInt;
	enum PVRSRV_HANDLE_TYPE eHandleType;
	enum PVRSRV_ERROR eError;

	eError = PVRSRVLookupHandleAnyType(psHandleBase, &hMHandleInt,
					   &eHandleType, hMHandle);
	if (eError != PVRSRV_OK)
		return eError;

	switch (eHandleType) {
	case PVRSRV_HANDLE_TYPE_MEM_INFO:
	case PVRSRV_HANDLE_TYPE_MEM_INFO_REF:
	case PVRSRV_HANDLE_TYPE_SHARED_SYS_MEM_INFO:
		{
			struct PVRSRV_KERNEL_MEM_INFO *psMemInfo =
			    (struct PVRSRV_KERNEL_MEM_INFO *)hMHandleInt;

			*phOSMemHandle = psMemInfo->sMemBlk.hOSMemHandle;

			break;
		}
	case PVRSRV_HANDLE_TYPE_SYNC_INFO:
		{
			struct PVRSRV_KERNEL_SYNC_INFO *psSyncInfo =
			    (struct PVRSRV_KERNEL_SYNC_INFO *)hMHandleInt;
			struct PVRSRV_KERNEL_MEM_INFO *psMemInfo =
			    psSyncInfo->psSyncDataMemInfoKM;

			*phOSMemHandle = psMemInfo->sMemBlk.hOSMemHandle;

			break;
		}
	case PVRSRV_HANDLE_TYPE_SOC_TIMER:
		{
			*phOSMemHandle = (void *)hMHandleInt;
			break;
		}
	default:
		return PVRSRV_ERROR_BAD_MAPPING;
	}

	return PVRSRV_OK;;
}
