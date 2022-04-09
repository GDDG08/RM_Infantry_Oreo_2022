/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Library\list_lib.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:54:52
 */

#include "list_lib.h"

/**
 * @brief          Initialize linked list
 * @param          pxList: Pointer to the initialized list structure
 * @retval         NULL
 */
void List_InitList(List_ListTypeDef* pxList, void* pxStruct) {
    if (pxList == NULL)
        return;

    pxList->typ_size = sizeof(pxStruct);
    List_NodeTypeDef* end_node = (List_NodeTypeDef*)malloc(sizeof(List_NodeTypeDef) + pxList->typ_size);

    end_node->node_typedef = pxStruct;
    end_node->pxContainer = pxList;
    end_node->pxNext = end_node;
    end_node->pxPrevious = end_node;

    pxList->end_node = end_node;

    pxList->current_node = 1;
    pxList->items_number = 1;
}

/**
 * @brief          Insert a node at the end of the list
 * @param          pxList: Pointer to the initialized list structure
 * @param          pxStruct :Inserted struct pointer
 * @retval         NULL
 */
void List_InsertEnd(List_ListTypeDef* pxList, void* pxStruct) {
    if ((pxList == NULL) || (pxStruct == NULL))
        return;

    List_NodeTypeDef* new_node = (List_NodeTypeDef*)malloc(sizeof(List_NodeTypeDef) + pxList->typ_size);
    new_node->node_typedef = pxStruct;
    new_node->pxContainer = pxList;

    new_node->pxPrevious = pxList->end_node->pxPrevious;
    new_node->pxNext = pxList->end_node;

    pxList->end_node->pxPrevious = new_node;
    new_node->pxPrevious->pxNext = new_node;

    (pxList->items_number)++;
}

/**
 * @brief          Get the nth node pointer
 * @param          pxList: Pointer to the insert list structure
 * @param          pxNewListItem :Node pointer
 * @retval         nth node pointer
 */
List_NodeTypeDef* List_GetListPtr(List_ListTypeDef* pxList, uint32_t num) {
    if (pxList == NULL)
        return NULL;
    if (num == 0) {
        num++;
    }
    List_NodeTypeDef* pxNode;

    pxNode = pxList->end_node;
    for (int i = 0; i < num; i++) {
        pxNode = pxNode->pxNext;
    }
    pxList->current_node = num % pxList->items_number;
    if (pxList->current_node < 1)
        pxList->current_node = pxList->items_number;
    return pxNode;
}

/**
 * @brief          Get the nth node data pointer
 * @param          pxList: Pointer to the insert list structure
 * @param          pxNewListItem :Node pointer
 * @retval         nth node pointer
 */
void* List_GetListDataPtr(List_ListTypeDef* pxList, uint32_t num) {
    if (pxList == NULL)
        return NULL;
    List_NodeTypeDef* node = List_GetListPtr(pxList, num);
    if (node == NULL)
        return NULL;

    return node->node_typedef;
}

/**
 * @brief          Get the current node data pointer
 * @param          pxList: Pointer to the insert list structure
 * @param          pxNewListItem :Node pointer
 * @retval         nth node pointer
 */
void* List_GetCurrentListDataPtr(List_ListTypeDef* pxList) {
    if (pxList == NULL)
        return NULL;
    List_NodeTypeDef* node = List_GetListPtr(pxList, pxList->current_node);
    if (node == NULL)
        return NULL;

    return node->node_typedef;
}

/**
 * @brief          Get the Next node pointer
 * @param          pxList: Pointer to the insert list structure
 * @param          pxNewListItem :Node pointer
 * @retval         nth node pointer
 */
void* List_GetNextListDataPtr(List_ListTypeDef* pxList) {
    if (pxList == NULL)
        return NULL;
    List_NodeTypeDef* node = List_GetListPtr(pxList, pxList->current_node + 1);
    if (node == NULL)
        return NULL;

    return node->node_typedef;
}

/**
 * @brief          Get the Previous node pointer
 * @param          pxList: Pointer to the insert list structure
 * @param          pxNewListItem :Node pointer
 * @retval         nth node pointer
 */
void* List_GetPreviousListDataPtr(List_ListTypeDef* pxList) {
    if (pxList == NULL)
        return NULL;
    List_NodeTypeDef* node = List_GetListPtr(pxList, pxList->current_node - 1);
    if (node == NULL)
        return NULL;

    return node->node_typedef;
}

/**
 * @brief          Insert a node into the linked list
 * @param          pxList: Pointer to the insert list structure
 * @param          pxStruct :Inserted struct pointer
 * @param          pxNewListItem :Inserted node pointer
 * @retval         NULL
 */
void List_Insert(List_ListTypeDef* pxList, void* pxStruct, uint32_t insert_num) {
    if ((pxList == NULL) || (pxStruct == NULL))
        return;

    if (insert_num >= pxList->items_number) {
        List_InsertEnd(pxList, pxStruct);
        return;
    }

    List_NodeTypeDef* new_node = (List_NodeTypeDef*)malloc(sizeof(List_NodeTypeDef) + pxList->typ_size);
    new_node->node_typedef = pxStruct;
    new_node->pxContainer = pxList;

    List_NodeTypeDef* pxIterator = List_GetListPtr(pxList, insert_num);
    if (pxIterator == NULL)
        return;

    new_node->pxPrevious = pxIterator->pxPrevious;
    new_node->pxNext = pxIterator;

    pxIterator->pxPrevious->pxNext = new_node;
    pxIterator->pxPrevious = new_node;

    (pxList->items_number)++;
}

/**
 * @brief          Remove a node
 * @param          pxNewListItem :Removed node
 * @retval         NULL
 */
uint32_t List_Remove(List_ListTypeDef* pxList, uint32_t deleat_num) {
    if (pxList == NULL)
        return NULL;

    if (deleat_num >= pxList->items_number) {
        List_Remove(pxList, (pxList->items_number - 1));
        return 0;
    }
    List_NodeTypeDef* pxItemToRemove = List_GetListPtr(pxList, deleat_num);
    if (pxItemToRemove == pxList->end_node)
        return 1;

    pxItemToRemove->pxNext->pxPrevious = pxItemToRemove->pxPrevious;
    pxItemToRemove->pxPrevious->pxNext = pxItemToRemove->pxNext;

    pxItemToRemove->pxContainer = NULL;
    pxItemToRemove->node_typedef = NULL;
    pxItemToRemove->pxPrevious = NULL;
    pxItemToRemove->pxNext = NULL;

    free(pxItemToRemove);
    (pxList->items_number)--;

    return pxList->items_number;
}
