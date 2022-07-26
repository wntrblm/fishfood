#include "littleg.h"
#include <stdio.h>
#include <string.h>

enum ParseState {
    PARSING_EOL,
    PARSING_FIELD_NAME,
    PARSING_FIELD_VALUE,
    PARSING_COMMENT,
};

struct Parser {
    enum ParseState state;
    bool valid;
    struct lilg_Command command;
    char field;
    struct lilg_Decimal value;
    bool sign;
    bool has_decimal;
};

static struct Parser parser = {};

static void parse_reset_field() {
    parser.state = PARSING_FIELD_NAME;
    parser.field = '\0';
    parser.value = (struct lilg_Decimal){};
    parser.sign = true;
    parser.has_decimal = false;
}

static void parse_reset() {
    parse_reset_field();
    parser.command = (struct lilg_Command){};
    parser.valid = false;
}

static void parse_end_field() {
    if (parser.state != PARSING_FIELD_VALUE)
        return;

    parser.value.set = true;
    if (!parser.sign)
        parser.value.real = -parser.value.real;

    switch (parser.field) {
        case 'G':
            parser.command.G = parser.value;
            break;
        case 'M':
            parser.command.M = parser.value;
            break;
        case 'X':
            parser.command.X = parser.value;
            break;
        case 'Y':
            parser.command.Y = parser.value;
            break;
        case 'Z':
            parser.command.Z = parser.value;
            break;
    }

    parser.command.fields[parser.field - 'A'] = parser.value;
    parse_reset_field();
    parser.valid = true;
}

bool lilg_parse(struct lilg_Command* command, char c) {
    if (parser.state == PARSING_EOL) {
        parse_reset();
    }

    // Comments
    if (c == ';') {
        parser.state = PARSING_COMMENT;
        return false;
    }

    // EOL
    if (c == '\0' || c == '\n' || c == '\r') {
        parse_end_field();
        parser.state = PARSING_EOL;
        // A valid command is ready if any field was seen during this line.
        memcpy(command, &parser.command, sizeof(struct lilg_Command));
        return parser.valid;
    }

    switch (parser.state) {
        case PARSING_FIELD_NAME: {
            if (c == ' ') {
                break;
            }
            if (c >= 'a' && c <= 'z') {
                c = 'A' + (c - 'a');
            }
            if (c >= 'A' && c <= 'Z') {
                parser.state = PARSING_FIELD_VALUE;
                parser.field = c;
            } else {
                // Anything else is an error
                parse_reset();
                parser.state = PARSING_COMMENT;
            }
            break;
        }

        case PARSING_FIELD_VALUE: {
            if (c == '-') {
                parser.sign = false;
            } else if (c >= '0' && c <= '9') {
                if (!parser.has_decimal) {
                    parser.value.real = parser.value.real * 10 + (uint8_t)(c - '0');
                } else {
                    parser.value.frac = parser.value.frac * 10 + (uint8_t)(c - '0');
                    parser.value.exp--;
                }
            } else if (c == '.') {
                parser.has_decimal = true;
            } else if (c == ' ') {
                parse_end_field();
            } else {
                // Anything else is an error
                parse_reset();
                parser.state = PARSING_COMMENT;
            }
            break;
        }

        case PARSING_COMMENT: {
            return false;
        }

        default:
            return false;
    }

    return false;
}

void lilg_Command_print(struct lilg_Command* cmd) {
    printf("lilg_Command: \n");
    for (size_t n = 0; n < 25; n++) {
        struct lilg_Decimal v = parser.command.fields[n];
        if (v.set) {
            printf("%c:   %0.2f\n", (char)('A' + n), lilg_Decimal_to_float(parser.command.fields[n]));
        }
    }
}
