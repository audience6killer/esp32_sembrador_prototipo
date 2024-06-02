#ifndef MAIN_TASKS_COMMON_H
#define MAIN_TASKS_COMMON_H

// Wifi application task config for the FreeRTOS task
#define WIFI_APP_TASK_STACK_SIZE        4096
#define WIFI_APP_TASK_PRIORITY          5
#define WIFI_APP_TASK_CORE_ID           0

// HTTP server task 
#define HTTP_SERVER_TASK_STACK_SIZE     8192*4
#define HTTP_SERVER_TASK_PRIORITY       4
#define HTTP_SERVER_CORE_ID             0

// Define HTTP server monitor task
#define HTTP_SERVER_MONITOR_TASK_STACK_SIZE     4096
#define HTTP_SERVER_MONITOR_TASK_PRIORITY       3
#define HTTP_SERVER_MONITOR_CORE_ID             0

// Define GPS UART task
#define GPS_UART_TASK_STACK_SIZE     4096
#define GPS_UART_TASK_PRIORITY       2
#define GPS_UART_CORE_ID             0
#endif