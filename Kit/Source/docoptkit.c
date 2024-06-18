#include "docoptkit.h"

const char help_message[] =
    "42 SmallSat Simulator\n"
    "\n"
    "Usage:\n"
    "  42 --version\n"
    "  42 -h | --help\n"
    "  42 [options]\n"
    "\n"
    "Options:\n"
    "  -h, --help                    Show this screen.\n"
    "  -d DIR --defaultdir=DIR       Top directory for Custom Input, Output, "
    "and Model Files\n"
    "  -i DIR --indir=DIR            Directory for Custom Input Files, "
    "overrides --defaultdir\n"
    "  -o DIR --outdir=DIR           Directory for Simulation Output FILES, "
    "overrides --defaultdir\n"
    "  -m DIR --modeldir=DIR         Directory for Custom Model Files, "
    "overrides --defaultdir\n"
    "  -g BOOL --graphics=BOOLEAN    Force enable (TRUE, 1) or disable (FALSE, "
    "0) graphics regardless of what is in the input file.\n"
    "\n"
    "";

const char usage_pattern[] = "Usage:\n"
                             "  42 -h | --help\n"
                             "  42 [options]";

Tokens tokens_new(int argc, char **argv) {
   Tokens ts = {argc, argv, 0, argv[0]};
   return ts;
}

Tokens *tokens_move(Tokens *ts) {
   if (ts->i < ts->argc) {
      ts->current = ts->argv[++ts->i];
   }
   if (ts->i == ts->argc) {
      ts->current = NULL;
   }
   return ts;
}

/*
 * ARGV parsing functions
 */

int parse_doubledash(Tokens *ts, Elements *elements) {
   // int n_commands = elements->n_commands;
   // int n_arguments = elements->n_arguments;
   // Command *commands = elements->commands;
   // Argument *arguments = elements->arguments;

   // not implemented yet
   // return parsed + [Argument(None, v) for v in tokens]
   return 0;
}

int parse_long(Tokens *ts, Elements *elements) {
   int i;
   int len_prefix;
   int n_options = elements->n_options;
   char *eq      = strchr(ts->current, '=');
   Option *option;
   Option *options = elements->options;

   len_prefix = (eq - (ts->current)) / sizeof(char);
   for (i = 0; i < n_options; i++) {
      option = &options[i];
      if (!strncmp(ts->current, option->olong, len_prefix))
         break;
   }
   if (i == n_options) {
      // TODO '%s is not a unique prefix
      fprintf(stderr, "%s is not recognized\n", ts->current);
      return 1;
   }
   tokens_move(ts);
   if (option->argcount) {
      if (eq == NULL) {
         if (ts->current == NULL) {
            fprintf(stderr, "%s requires argument\n", option->olong);
            return 1;
         }
         option->argument = ts->current;
         tokens_move(ts);
      } else {
         option->argument = eq + 1;
      }
   } else {
      if (eq != NULL) {
         fprintf(stderr, "%s must not have an argument\n", option->olong);
         return 1;
      }
      option->value = true;
   }
   return 0;
}

int parse_shorts(Tokens *ts, Elements *elements) {
   char *raw;
   int i;
   int n_options = elements->n_options;
   Option *option;
   Option *options = elements->options;

   raw = &ts->current[1];
   tokens_move(ts);
   while (raw[0] != '\0') {
      for (i = 0; i < n_options; i++) {
         option = &options[i];
         if (option->oshort != NULL && option->oshort[1] == raw[0])
            break;
      }
      if (i == n_options) {
         // TODO -%s is specified ambiguously %d times
         fprintf(stderr, "-%c is not recognized\n", raw[0]);
         return 1;
      }
      raw++;
      if (!option->argcount) {
         option->value = true;
      } else {
         if (raw[0] == '\0') {
            if (ts->current == NULL) {
               fprintf(stderr, "%s requires argument\n", option->oshort);
               return 1;
            }
            raw = ts->current;
            tokens_move(ts);
         }
         option->argument = raw;
         break;
      }
   }
   return 0;
}

int parse_argcmd(Tokens *ts, Elements *elements) {
   int i;
   int n_commands = elements->n_commands;
   // int n_arguments = elements->n_arguments;
   Command *command;
   Command *commands = elements->commands;
   // Argument *arguments = elements->arguments;

   for (i = 0; i < n_commands; i++) {
      command = &commands[i];
      if (!strcmp(command->name, ts->current)) {
         command->value = true;
         tokens_move(ts);
         return 0;
      }
   }
   // not implemented yet, just skip for now
   // parsed.append(Argument(None, tokens.move()))
   /*fprintf(stderr, "! argument '%s' has been ignored\n", ts->current);
   fprintf(stderr, "  '");
   for (i=0; i<ts->argc ; i++)
       fprintf(stderr, "%s ", ts->argv[i]);
   fprintf(stderr, "'\n");*/
   tokens_move(ts);
   return 0;
}

int parse_args(Tokens *ts, Elements *elements) {
   int ret;

   while (ts->current != NULL) {
      if (strcmp(ts->current, "--") == 0) {
         ret = parse_doubledash(ts, elements);
         if (!ret)
            break;
      } else if (ts->current[0] == '-' && ts->current[1] == '-') {
         ret = parse_long(ts, elements);
      } else if (ts->current[0] == '-' && ts->current[1] != '\0') {
         ret = parse_shorts(ts, elements);
      } else
         ret = parse_argcmd(ts, elements);
      if (ret)
         return ret;
   }
   return 0;
}

int elems_to_args(Elements *elements, DocoptArgs *args, bool help) {
   Command *command;
   Argument *argument;
   Option *option;
   int i;

   // fix gcc-related compiler warnings (unused)
   (void)command;
   (void)argument;

   /* options */
   for (i = 0; i < elements->n_options; i++) {
      option = &elements->options[i];
      if (help && option->value && !strcmp(option->olong, "--help")) {
         printf("%s", args->help_message);
         return 1;
      } else if (!strcmp(option->olong, "--help")) {
         args->help = option->value;
      } else if (!strcmp(option->olong, "--defaultdir")) {
         if (option->argument)
            args->defaultdir = option->argument;
      } else if (!strcmp(option->olong, "--graphics")) {
         if (option->argument)
            args->graphics = option->argument;
      } else if (!strcmp(option->olong, "--indir")) {
         if (option->argument)
            args->indir = option->argument;
      } else if (!strcmp(option->olong, "--modeldir")) {
         if (option->argument)
            args->modeldir = option->argument;
      } else if (!strcmp(option->olong, "--outdir")) {
         if (option->argument)
            args->outdir = option->argument;
      }
   }
   /* commands */
   for (i = 0; i < elements->n_commands; i++) {
      command = &elements->commands[i];
   }
   /* arguments */
   for (i = 0; i < elements->n_arguments; i++) {
      argument = &elements->arguments[i];
   }
   return 0;
}

/*
 * Main docopt function
 */

DocoptArgs docopt(int argc, char *argv[], bool help) {
   DocoptArgs args = {0,    0,    NULL,          NULL,        NULL,
                      NULL, NULL, usage_pattern, help_message};
   Tokens ts;
   Command commands[]   = {};
   Argument arguments[] = {};
   Option options[]     = {
       {"-h", "--help", 0, 0, NULL},     {"-d", "--defaultdir", 1, 0, NULL},
       {"-g", "--graphics", 1, 0, NULL}, {"-i", "--indir", 1, 0, NULL},
       {"-m", "--modeldir", 1, 0, NULL}, {"-o", "--outdir", 1, 0, NULL}};
   Elements elements = {0, 0, 6, commands, arguments, options};

   ts = tokens_new(argc, argv);
   if (parse_args(&ts, &elements))
      exit(EXIT_FAILURE);
   if (elems_to_args(&elements, &args, help))
      exit(EXIT_SUCCESS);
   return args;
}
