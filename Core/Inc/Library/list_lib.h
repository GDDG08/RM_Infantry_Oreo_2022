/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \infantry_-neptune\Core\Inc\Library\list_lib.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-23 21:39:06
 */

#ifndef LIST_LIB_H
#define LIST_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#include <stdlib.h>
#include "math_alg.h"

struct list;
typedef struct list_node {
    void* node_typedef;

    struct list_node* pxNext;
    struct list_node* pxPrevious;
    struct list* pxContainer;
} List_NodeTypeDef;

typedef struct list {
    uint32_t current_node;

    uint32_t typ_size;
    uint32_t items_number;
    List_NodeTypeDef* end_node;
} List_ListTypeDef;

void List_InitList(List_ListTypeDef* pxList, void* pxStruct);
void List_InsertEnd(List_ListTypeDef* pxList, void* pxStruct);
List_NodeTypeDef* List_GetListPtr(List_ListTypeDef* pxList, uint32_t num);
void* List_GetListDataPtr(List_ListTypeDef* pxList, uint32_t num);
void* List_GetNextListDataPtr(List_ListTypeDef* pxList);
void* List_GetCurrentListDataPtr(List_ListTypeDef* pxList);
void* List_GetPreviousListDataPtr(List_ListTypeDef* pxList);
void List_Insert(List_ListTypeDef* pxList, void* pxStruct, uint32_t insert_num);
uint32_t List_Remove(List_ListTypeDef* pxList, uint32_t deleat_num);

#ifdef __cplusplus
}
#endif

#endif
