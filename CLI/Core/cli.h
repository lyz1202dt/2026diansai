#ifndef __CLI_H__
#define __CLI_H__

#include <stdint.h>
#include "Driver/list/mylist.h"

typedef struct CLI CLI_t;

typedef int(*ExecutFunc)(CLI_t* cli,int argc,char** argv);

typedef struct{
    char* name;
    ExecutFunc cmd;
}Command_t;

struct CLI{
    void(*putchar)(char ch);
    char(*getchar)();
    MyList_t *cmd_list;          //命令列表
    char printfBuffer[128];
};

CLI_t* CLICreate();
int CLIAddCommand(CLI_t* cli,Command_t cmd);
void CLIDelete();
void CLIRun(CLI_t* cli);

void printf_cli(CLI_t* cli, const char* fmt, ...);
int scanf_cli(CLI_t* cli, const char* fmt, ...);

#endif
