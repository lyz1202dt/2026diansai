#include "CLI/App/app.h"

int __help(CLI_t* cli,int argc,char** argv)
{
    printf_cli(cli, "This is help cmd\n");
    return 0;
}
