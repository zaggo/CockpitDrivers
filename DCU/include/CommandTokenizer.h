#ifndef COMMANDTOKENIZER_H
#define COMMANDTOKENIZER_H
#include <stddef.h>

// Splits `buffer` in place on runs of spaces into up to maxTokens
// null-terminated tokens (space bytes in `buffer` get overwritten with '\0').
// Leading/trailing/repeated spaces are ignored. Returns the token count.
inline size_t tokenizeCommands(char* buffer, char** tokens, size_t maxTokens) {
    size_t count = 0;
    char* p = buffer;
    while (*p == ' ') ++p;

    while (*p != '\0' && count < maxTokens) {
        tokens[count++] = p;
        while (*p != '\0' && *p != ' ') ++p;
        if (*p == ' ') {
            *p = '\0';
            ++p;
            while (*p == ' ') ++p;
        }
    }
    return count;
}

#endif // COMMANDTOKENIZER_H
