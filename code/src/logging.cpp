#include <fstream>
#include <format>
#include "../headers/logging.hpp"

void Logger::_format_output(const char* str, va_list ptr, FILE* log_file) {

    // char array to store token 
    char token[1000]; 
    // index of where to store the characters of str in token 
    int k = 0; 

    // parsing the formatted string 
    for (int i = 0; str[i] != '\0'; i++) { 

        token[k++] = str[i];

        if (str[i + 1] == '%' || str[i + 1] == '\0') {

            token[k] = '\0'; 
            k = 0;
            if (token[0] != '%') {

                fprintf(stdout, "%s", token); // printing the whole token if it is not a format specifier
                fprintf(log_file, "%s", token);

            } else { 

                int j = 1; 
                char ch1 = 0; 

                // this loop is required when printing formatted value like 0.2f, when ch1='f' loop ends 
                while ((ch1 = token[j++]) < 58) {}

                if (ch1 == 'i' || ch1 == 'd' || ch1 == 'u' || ch1 == 'h') { // for integers

                    int value = va_arg(ptr, int);
                    fprintf(stdout, token, value);
                    fprintf(log_file, token, value);

                } else if (ch1 == 'c') { // for chars

                    char value = va_arg(ptr, int);
                    fprintf(stdout, token, value);
                    fprintf(log_file, token, value);

                } else if (ch1 == 'f') { // for float values 

                    double value = va_arg(ptr, double);
                    fprintf(stdout, token, value);
                    fprintf(log_file, token, value);

                } else if (ch1 == 'l') { // for long values

                    char ch2 = token[2];

                    if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long int values 

                        long value = va_arg(ptr, long);
                        fprintf(stdout, token, value);
                        fprintf(log_file, token, value);

                    } else if (ch2 == 'f') {  // for double values 

                        double value = va_arg(ptr, double);
                        fprintf(stdout, token, value);
                        fprintf(log_file, token, value);

                    } 

                } else if (ch1 == 'L') { 

                    char ch2 = token[2]; 

                    if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long long int values 

                        long long value = va_arg(ptr, long long);
                        fprintf(stdout, token, value);
                        fprintf(log_file, token, value);

                    } 

                    else if (ch2 == 'f') { // for long double values 

                        long double value = va_arg(ptr, long double);
                        fprintf(stdout, token, value);
                        fprintf(log_file, token, value);

                    } 

                } else if (ch1 == 's') { // for string values 

                    char* value = va_arg(ptr, char*);
                    fprintf(stdout, token, value);
                    fprintf(log_file, token, value);

                } else { // print the whole token if no case is matched 

                    fprintf(stdout, "%s", token);
                    fprintf(log_file, "%s", token);

                } 

            } 
        
        }

    }
}

Logger::Logger(const std::string run_title, const std::string log_name) {

    const char* c_log_name = log_name.c_str();
    snprintf(log_file_name, 100, "%s/%s/%s", HOME_DIR, LOG_DIR ,c_log_name);
    FILE* log_file = fopen(log_file_name, "a");
    fprintf(log_file, "\n--------------------------------\n%s\n--------------------------------\n", run_title.c_str());
    fclose(log_file);

}

void Logger::print_info(const char* str, ...) {

    #if VERBOSE < 10
    return;
    #endif

    // logging file
    FILE* log_file = fopen(log_file_name, "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[92m\033[1m[ INFO  ]:\033[0m ");
    fprintf(log_file, "[ INFO  ]: ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

}

void Logger::print_warn(const char* str, ...) {

    #if VERBOSE < 1
    return;
    #endif

    // logging file
    FILE* log_file = fopen(log_file_name, "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[93m\033[1m[ WARN  ]:\033[0m ");
    fprintf(log_file, "[ WARN  ]: ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

}

void Logger::raise_error(const char* str, ...) {

    // logging file
    FILE* log_file = fopen(log_file_name, "a");

    // initializing list pointer 
    va_list ptr; 
    va_start(ptr, str);

    fprintf(stdout, "\033[91m\033[1m[ ERROR ]:\033[0m ");
    fprintf(log_file, "[ ERROR ]: ");

    // printing and logging the formatted message
    _format_output(str, ptr, log_file);

    fprintf(stdout, "\n");
    fprintf(log_file, "\n");
    
    fclose(log_file);

    // ending traversal 
    va_end(ptr);

    exit(1);

}