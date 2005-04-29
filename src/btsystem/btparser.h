#ifndef _PARSER_H
#define _PARSER_H
int parseFile(char *fn);
int parseGetVal(int type, char *str, void *loc);
enum{INT,LONG,DOUBLE,STRING,VECTOR};
#endif /* _PARSER_H */
