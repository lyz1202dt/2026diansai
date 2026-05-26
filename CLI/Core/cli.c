#include "CLI/Core/cli.h"
#include <stdlib.h>
#include <string.h>

#define CLI_INPUT_BUFFER_SIZE 128
#define CLI_MAX_ARGC          16

static CLI_t *g_cli_instance = NULL;

static uint32_t CLICommandMatch(void *user, void *dst)
{
    const char *name = (const char *)user;
    Command_t *cmd = (Command_t *)dst;

    if ((name == NULL) || (cmd == NULL) || (cmd->name == NULL)) {
        return 0U;
    }

    return (uint32_t)(strcmp(name, cmd->name) == 0);
}

static int CLIParseArgs(char *line, char **argv, int max_argc)
{
    int argc = 0;
    char *token = NULL;

    if ((line == NULL) || (argv == NULL) || (max_argc <= 0)) {
        return 0;
    }

    token = strtok(line, " ");
    while ((token != NULL) && (argc < max_argc)) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }

    return argc;
}

void printf_cli(CLI_t* cli, const char* fmt, ...)
{
	va_list args;
	int len;

	if ((cli == NULL) || (fmt == NULL))
	{
		return;
	}

	va_start(args, fmt);
	len = vsnprintf(cli->printfBuffer, sizeof(cli->printfBuffer), fmt, args);
	va_end(args);

	if (len <= 0)
	{
		return;
	}

	if ((size_t) len >= sizeof(cli->printfBuffer))
	{
		len = (int) (sizeof(cli->printfBuffer) - 1);
	}

    for(int i=0;i<len;i++)
        cli->putchar(cli->printfBuffer[i]);
}

CLI_t* CLICreate()
{
    CLI_t *cli = (CLI_t *)malloc(sizeof(CLI_t));
    if (cli == NULL) {
        return NULL;
    }

    memset(cli, 0, sizeof(CLI_t));

    cli->cmd_list = ListCreate(sizeof(Command_t));
    if (cli->cmd_list == NULL) {
        free(cli);
        return NULL;
    }

    g_cli_instance = cli;

    return cli;
}

int CLIAddCommand(CLI_t* cli, Command_t cmd)
{
    if ((cli == NULL) || (cli->cmd_list == NULL) || (cmd.name == NULL) || (cmd.cmd == NULL)) {
        return -1;
    }

    if (ListFind(cli->cmd_list, cmd.name, CLICommandMatch) != NULL) {
        return -1;
    }

    return ListAddElement(cli->cmd_list, &cmd);
}

void CLIDelete()
{
    if (g_cli_instance == NULL) {
        return;
    }

    if (g_cli_instance->cmd_list != NULL) {
        ListRemove(g_cli_instance->cmd_list);
        free(g_cli_instance->cmd_list);
        g_cli_instance->cmd_list = NULL;
    }

    free(g_cli_instance);
    g_cli_instance = NULL;
}

void CLIRun(CLI_t* cli)
{
    char line_buffer[CLI_INPUT_BUFFER_SIZE];
    char *argv[CLI_MAX_ARGC];
    Command_t *cmd = NULL;
    int argc = 0;
    int index = 0;

    if ((cli == NULL) || (cli->getchar == NULL) || (cli->cmd_list == NULL)) {
        return;
    }

    memset(line_buffer, 0, sizeof(line_buffer));

    while (index < (CLI_INPUT_BUFFER_SIZE - 1)) {
        char ch = cli->getchar();

        if ((ch == '\r') || (ch == '\n')) {
            if (cli->putchar != NULL) {
                cli->putchar('\r');
                cli->putchar('\n');
            }
            break;
        }

        if ((ch == '\b') || (ch == 0x7FU)) {
            if (index > 0) {
                index--;
                line_buffer[index] = '\0';
                if (cli->putchar != NULL) {
                    cli->putchar('\b');
                    cli->putchar(' ');
                    cli->putchar('\b');
                }
            }
            continue;
        }

        line_buffer[index++] = ch;
        if (cli->putchar != NULL) {
            cli->putchar(ch);
        }
    }

    if (index == 0) {
        return;
    }

    line_buffer[index] = '\0';
    argc = CLIParseArgs(line_buffer, argv, CLI_MAX_ARGC);
    if (argc <= 0) {
        return;
    }

    cmd = (Command_t *)ListFind(cli->cmd_list, argv[0], CLICommandMatch);
    if ((cmd == NULL) || (cmd->cmd == NULL)) {
        return;
    }

    cmd->cmd(argc, argv);
}
