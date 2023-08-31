
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifndef __DOCOPTKIT_H__
#define __DOCOPTKIT_H__

typedef struct DocoptArgs{
    /* options without arguments */
    int help;
    int version;
    /* options with arguments */
    char *defaultdir;
    char *graphics;
    char *indir;
    char *modeldir;
    char *outdir;
    /* special */
    const char *usage_pattern;
    const char *help_message;
} DocoptArgs;

typedef struct {
    const char *name;
    bool value;
} Command;

typedef struct {
    const char *name;
    char *value;
    char **array;
} Argument;

typedef struct {
    const char *oshort;
    const char *olong;
    bool argcount;
    bool value;
    char *argument;
} Option;

typedef struct {
    int n_commands;
    int n_arguments;
    int n_options;
    Command *commands;
    Argument *arguments;
    Option *options;
} Elements;


/*
 * Tokens object
 */

typedef struct Tokens {
    int argc;
    char **argv;
    int i;
    char *current;
} Tokens;

Tokens tokens_new(int argc, char **argv);

Tokens* tokens_move(Tokens *ts);

int parse_doubledash(Tokens *ts, Elements *elements);

int parse_long(Tokens *ts, Elements *elements);

int parse_shorts(Tokens *ts, Elements *elements);

int parse_argcmd(Tokens *ts, Elements *elements);

int parse_args(Tokens *ts, Elements *elements);

int elems_to_args(Elements *elements, DocoptArgs *args, bool help);

DocoptArgs docopt(int argc, char *argv[], bool help);

#endif /* __DOCOPTKIT_H__ */
