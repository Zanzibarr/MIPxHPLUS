#include "../include/utils.hpp"

namespace my {

    // ##################################################################### //
    // ############################## BITFIELD ############################# //
    // ##################################################################### //

    BitField::BitField(unsigned int size, bool full_flag) {
        this -> size_ = size;
        this -> field_ = std::vector<char>((size+7)/8, full_flag ? (char)~0u : 0);
        if (full_flag && this -> size_ % 8 != 0) this->field_[this -> field_.size() - 1] &= (1u << this -> size_ % 8) - 1;          // mask for the bits outside the size
    }

    BitField::BitField(const BitField& other_bitfield) {
        this -> size_ = other_bitfield.size_;
        this -> field_ = std::vector<char>(other_bitfield.field_);
    }

    void BitField::set(unsigned int i) {

        #if HPLUS_INTCHECK
        assert(i < this -> size_, "BitField::set failed.");
        #endif
        this -> field_[i/8] |= (1 << i%8);

    }

    void BitField::unset(const unsigned int i) {

        #if HPLUS_INTCHECK
        assert(i < this -> size_, "BitField::unset failed.");
        #endif
        this -> field_[i/8] &= ~(1 << i%8);

    }

    bool BitField::operator[](const unsigned int i) const {

        #if HPLUS_INTCHECK
        assert(i < this -> size_, "BitField::[] failed.");
        #endif
        return this -> field_[i/8] & (1 << i%8);

    }

    unsigned int BitField::size() const { return this -> size_; }

    BitField BitField::operator&(const BitField& other_bitfield) const {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField::& failed.");
        #endif
        BitField new_bitfield(*this);
        for (unsigned int i = 0; i < this -> field_.size(); i++) new_bitfield.field_[i] &= other_bitfield.field_[i];
        return new_bitfield;

    }

    BitField& BitField::operator&=(const BitField& other_bitfield) {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField::&= failed.");
        #endif
        for (unsigned int i = 0; i < this -> field_.size(); i++) this -> field_[i] &= other_bitfield.field_[i];
        return *this;

    }

    BitField BitField::operator|(const BitField& other_bitfield) const {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField::| failed.");
        #endif
        BitField new_bitfield(*this);
        for (unsigned int i = 0; i < this -> field_.size(); i++) new_bitfield.field_[i] |= other_bitfield.field_[i];
        return new_bitfield;

    }

    BitField& BitField::operator|=(const BitField& other_bitfield) {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField::|= failed.");
        #endif
        for (unsigned int i = 0; i < this -> field_.size(); i++) this -> field_[i] |= other_bitfield.field_[i];
        return *this;

    }

    BitField BitField::operator-(const BitField& other_bitfield) const {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField::- failed.");
        #endif
        BitField new_bitfield(*this);
        for (unsigned int i = 0; i < this -> field_.size(); i++) new_bitfield.field_[i] &= ~other_bitfield.field_[i];
        return new_bitfield;

    }

    BitField& BitField::operator-=(const BitField& other_bitfield) {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField::-= failed.");
        #endif
        for (unsigned int i = 0; i < this -> field_.size(); i++) this -> field_[i] &= ~other_bitfield.field_[i];
        return *this;

    }

    BitField BitField::operator!() const {

        BitField new_bitfield(this -> size_);
        for (unsigned int i = 0; i < this -> field_.size(); i++) new_bitfield.field_[i] = ~this -> field_[i];
        if (this -> size_ % 8 != 0) new_bitfield.field_[this -> field_.size() - 1] &= (1u << this -> size_ % 8) - 1;        // mask for the bits outside the size
        return new_bitfield;

    }

    void BitField::clear() { this -> field_ = std::vector<char>((this -> size_+7)/8, 0); }

    bool BitField::operator==(const BitField& other_bitfield) const {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField:== failed.");
        #endif
        unsigned int i;
        for (i = 0; i < this -> field_.size() && this -> field_[i] == other_bitfield.field_[i]; i++);
        return i == this -> field_.size();

    }

    bool BitField::operator!=(const BitField& other_bitfield) const { return !((*this)==other_bitfield); }

    bool BitField::intersects(const BitField& other_bitfield) const {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField:intersects failed.");
        #endif
        unsigned int i;
        for (i = 0; i < this -> field_.size() && !(this -> field_[i] & other_bitfield.field_[i]); i++);
        return i != this -> field_.size();

    }

    bool BitField::contains(const BitField& other_bitfield) const {

        #if HPLUS_INTCHECK
        assert(this -> size_ == other_bitfield.size_, "BitField:contains failed.");
        #endif
        unsigned int i;
        for (i = 0; i < this -> field_.size() && !(~this -> field_[i] & other_bitfield.field_[i]); i++);
        return i == this -> field_.size();

    }

    std::vector<unsigned int> BitField::sparse() const {

        std::vector<unsigned int> ret;
        for (auto p : *this) ret.push_back(p);
        return ret;

    }

    BitField::operator std::string() const {

        std::string ret = "[";
        for (unsigned int i = 0; i < this -> size_; i++) ret.append(this -> operator[](i) ? "X" : " ");
        ret.append("]");
        return ret;

    }

    BitField::Iterator::Iterator(const BitField *bitField, unsigned int index) {

        bf_ = bitField;
        index_ = index;
        if (index_ < bf_->size() && !(*bf_)[index_]) ++(*this);

    }

    BitField::Iterator& BitField::Iterator::operator++()  {

        do {
            ++index_;
        } while (index_ < bf_ -> size() && !(*bf_)[index_]);

        return *this;

    }

    unsigned int BitField::Iterator::operator*() const { return index_; }

    bool BitField::Iterator::operator!=(const Iterator &other) const { return index_ != other.index_; }

    BitField::Iterator BitField::begin() const { return Iterator(this, 0); }

    BitField::Iterator BitField::end() const { return Iterator(this, size_); }

    // ##################################################################### //
    // ####################### SUBSET SEARCH BIN TREE ###################### //
    // ##################################################################### //

    SSBT::SSBT() { root = new treenode(); }

    void SSBT::add(unsigned int value, const BitField& set) {
        treenode* leaf = this -> root;
        for (unsigned int i = 0; i < set.size(); i++) {
            if (set[i]) {
                if (leaf -> r == nullptr) leaf -> r = new treenode();
                leaf = leaf -> r;
            } else {
                if (leaf -> l == nullptr) leaf -> l = new treenode();
                leaf = leaf -> l;
            }
        }
        (leaf -> v).push_back(value);
    }

    std::vector<unsigned int> SSBT::find_subsets(const BitField& set) {
        std::deque<treenode*> open_nodes;
        open_nodes.push_back(this -> root);
        for (unsigned int i = 0; i < set.size() && !open_nodes.empty(); i++) {
            unsigned int tmp = open_nodes.size();
            for (unsigned int _ = 0; _ < tmp; _++) {
                treenode* node = open_nodes.front();
                open_nodes.pop_front();
                if (set[i]) {
                    if (node -> l != nullptr) open_nodes.push_back(node -> l);
                    if (node -> r != nullptr) open_nodes.push_back(node -> r);
                } else {
                    if (node -> l != nullptr) open_nodes.push_back(node -> l);
                }
            }
        }
        std::vector<unsigned int> result;
        for (auto node : open_nodes) result.insert(result.end(), node -> v.begin(), node -> v.end());
        return result;
    }

    SSBT::~SSBT() {
        std::deque<treenode*> remaining_nodes;
        remaining_nodes.push_back(this -> root);
        while (!remaining_nodes.empty()) {
            treenode* node = remaining_nodes.front();
            remaining_nodes.pop_front();
            if (node -> l != nullptr) remaining_nodes.push_back(node -> l);
            if (node -> r != nullptr) remaining_nodes.push_back(node -> r);
            delete node;
        }
    }

    // ##################################################################### //
    // ############################### LOGGER ############################## //
    // ##################################################################### //

    void Logger::format_output_(const char* str, va_list ptr, FILE* log_file, const bool show_time, bool error) {

        if (show_time) {
            double elapsed_time = HPLUS_env.get_time();
            fprintf(error ? stderr : stdout, "%8.3f : ", elapsed_time);
            if (HPLUS_env.log) fprintf(log_file, "%8.3f : ", elapsed_time);
        }

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

                    fprintf(error ? stderr : stdout, "%s", token); // printing the whole token if it is not a format specifier
                    if (HPLUS_env.log) fprintf(log_file, "%s", token);

                } else {

                    int j = 1;
                    char ch1;

                    // this loop is required when printing formatted value like 0.2f, when ch1='f' loop ends
                    while ((ch1 = token[j++]) < 58) {}

                    if (ch1 == 'i' || ch1 == 'd' || ch1 == 'u' || ch1 == 'h') { // for integers

                        int value = va_arg(ptr, int);
                        fprintf(error ? stderr : stdout, token, value);
                        if (HPLUS_env.log) fprintf(log_file, token, value);

                    } else if (ch1 == 'c') { // for chars

                        char value = va_arg(ptr, int);
                        fprintf(error ? stderr : stdout, token, value);
                        if (HPLUS_env.log) fprintf(log_file, token, value);

                    } else if (ch1 == 'f') { // for float values

                        double value = va_arg(ptr, double);
                        fprintf(error ? stderr : stdout, token, value);
                        if (HPLUS_env.log) fprintf(log_file, token, value);

                    } else if (ch1 == 'l') { // for long values

                        char ch2 = token[2];

                        if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long int values

                            long value = va_arg(ptr, long);
                            fprintf(error ? stderr : stdout, token, value);
                            if (HPLUS_env.log) fprintf(log_file, token, value);

                        } else if (ch2 == 'f') {  // for double values

                            double value = va_arg(ptr, double);
                            fprintf(error ? stderr : stdout, token, value);
                            if (HPLUS_env.log) fprintf(log_file, token, value);

                        }

                    } else if (ch1 == 'L') {

                        char ch2 = token[2];

                        if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long long int values

                            long long value = va_arg(ptr, long long);
                            fprintf(error ? stderr : stdout, token, value);
                            if (HPLUS_env.log) fprintf(log_file, token, value);

                        }

                        else if (ch2 == 'f') { // for long double values

                            long double value = va_arg(ptr, long double);
                            fprintf(error ? stderr : stdout, token, value);
                            if (HPLUS_env.log) fprintf(log_file, token, value);

                        }

                    } else if (ch1 == 's') { // for string values

                        char* value = va_arg(ptr, char*);
                        fprintf(error ? stderr : stdout, token, value);
                        if (HPLUS_env.log) fprintf(log_file, token, value);

                    } else { // print the whole token if no case is matched

                        fprintf(error ? stderr : stdout, "%s", token);
                        if (HPLUS_env.log) fprintf(log_file, "%s", token);

                    }

                }

            }

        }
    }

    Logger::Logger(const std::string& run_title, const std::string& log_name) {

        this -> log_file_name_ = log_name;
        if (HPLUS_env.log)  {
            FILE* log_file = fopen(this -> log_file_name_.c_str(), "a");
            fprintf(log_file, "\n%s\nRUN_NAME: %s\n%s\n", THICK_LINE, run_title.c_str(), THICK_LINE);
            fclose(log_file);
        }

    }

    void Logger::print(const char* str, ...) const {

        #if HPLUS_VERBOSE == 0
        return;
        #endif

        // logging file
        FILE* log_file = nullptr;
        if (HPLUS_env.log) log_file = fopen(this -> log_file_name_.c_str(), "a");

        // initializing list pointer
        va_list ptr;
        va_start(ptr, str);

        // printing and logging the formatted message
        format_output_(str, ptr, log_file, false);

        fprintf(stdout, "\n");
        if (HPLUS_env.log) fprintf(log_file, "\n");

        if (HPLUS_env.log) fclose(log_file);

        // ending traversal
        va_end(ptr);

    }

    void Logger::print_info(const char* str, ...) const {

        #if HPLUS_VERBOSE < 10
        return;
        #endif

        // logging file
        FILE* log_file = nullptr;
        if (HPLUS_env.log) log_file = fopen(this -> log_file_name_.c_str(), "a");

        // initializing list pointer
        va_list ptr;
        va_start(ptr, str);

        fprintf(stdout, "\033[92m\033[1m[ INFO  ]\033[0m -- ");
        if (HPLUS_env.log) fprintf(log_file, "[ INFO  ] -- ");

        // printing and logging the formatted message
        format_output_(str, ptr, log_file);

        fprintf(stdout, "\n");
        if (HPLUS_env.log) fprintf(log_file, "\n");

        if (HPLUS_env.log) fclose(log_file);

        // ending traversal
        va_end(ptr);

    }

    void Logger::print_warn(const char* str, ...) const {

        #if HPLUS_WARN == 0
        return;
        #endif

        // logging file
        FILE* log_file = nullptr;
        if (HPLUS_env.log) log_file = fopen(this -> log_file_name_.c_str(), "a");

        // initializing list pointer
        va_list ptr;
        va_start(ptr, str);

        fprintf(stdout, "\033[93m\033[1m[ WARN  ]\033[0m -- ");
        if (HPLUS_env.log) fprintf(log_file, "[ WARN  ] -- ");

        // printing and logging the formatted message
        format_output_(str, ptr, log_file);

        fprintf(stdout, "\n");
        if (HPLUS_env.log) fprintf(log_file, "\n");

        if (HPLUS_env.log) fclose(log_file);

        // ending traversal
        va_end(ptr);

    }

    void Logger::raise_error(const char* str, ...) const {

        // logging file
        FILE* log_file = nullptr;
        if (HPLUS_env.log) log_file = fopen(this -> log_file_name_.c_str(), "a");

        // initializing list pointer
        va_list ptr;
        va_start(ptr, str);

        fprintf(stderr, "\033[91m\033[1m[ ERROR ]\033[0m -- ");
        if (HPLUS_env.log) fprintf(log_file, "[ ERROR ] -- ");

        // printing and logging the formatted message
        format_output_(str, ptr, log_file, true, true);

        fprintf(stderr, "\n");
        if (HPLUS_env.log) fprintf(log_file, "\n");

        if (HPLUS_env.log) fclose(log_file);

        // ending traversal
        va_end(ptr);

        exit(1);

    }

    // ##################################################################### //
    // ############################ MY NAMESPACE ########################### //
    // ##################################################################### //

    std::vector<std::string> split_string(const std::string& str, const char del) {

        std::vector<std::string> tokens;

        std::string tmp;
        for (int i = 0; i < str.length(); i++) {
            if (str[i] == del) {
                if (!tmp.empty()) tokens.push_back(tmp);
                tmp = "";
            } else tmp += str[i];
        }
        if (!tmp.empty()) tokens.push_back(tmp);

        return tokens;

    }

    void assert(const bool condition, const std::string message) { if (!condition) HPLUS_env.logger.raise_error("%s - Assert check failed.", message.c_str()); }

    bool isint(const std::string& str, const int from, const int to) {

        try {
            int num = stoi(str);
            return num >= from && num <= to;
        } catch (std::invalid_argument&) { return false; }

    }

    void todo() { lraise_error("UNIMPLEMENTED."); }

    void pause(const std::string message) {

        unsigned int test = 0;
        std::cout << message << "(1 to exit, 0 to continue): ";
        std::cin >> test;
        if (test > 0) exit(1);

    }

}

// ##################################################################### //
// ############################## GLOBALS ############################## //
// ##################################################################### //

HPLUS_Environment HPLUS_env;
HPLUS_Statistics HPLUS_stats;

void HPLUS_Environment::start_timer() { this -> timer = std::chrono::steady_clock::now(); }
double HPLUS_Environment::get_time() const { return ((double) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this -> timer).count()) / 1000; }

void HPLUS_Statistics::print() const {

    #if HPLUS_VERBOSE < 5
    return;
    #endif

    lprint("\n\n--------------------------------------------------");
    lprint("-----------------   Statistics   -----------------");
    lprint("--------------------------------------------------\n");
    HPLUS_env.logger.print(" >>  Parsing time                  %10.3fs  <<", this->parsing_time);
    HPLUS_env.logger.print(" >>  Model optimization time       %10.3fs  <<", this->opt_time);
    HPLUS_env.logger.print(" >>  Heuristic time                %10.3fs  <<", this->heuristic_time);
    HPLUS_env.logger.print(" >>  Model building time           %10.3fs  <<", this->build_time);
    HPLUS_env.logger.print(" >>  CPLEX execution time          %10.3fs  <<", this->exec_time);
    HPLUS_env.logger.print(" >>  Total time                    %10.3fs  <<", this->total_time);
    lprint("\n\n");

}