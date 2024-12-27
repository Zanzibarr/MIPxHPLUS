#include "../include/utils.hpp"

my::logger mylog;
my::time_keeper timer;

namespace my {

    // ##################################################################### //
    // ############################# BINARY SET ############################ //
    // ##################################################################### //

    binary_set::binary_set(size_t capacity, bool full_flag) {
        this -> capacity_ = capacity;
        this -> set_ = std::vector<char>((this -> capacity_+7)/8, full_flag ? (char)~0u : 0);
        if (full_flag && this -> capacity_ % 8 != 0) this -> set_[this -> set_.size() - 1] &= (1u << this -> capacity_ % 8) - 1;          // mask for the bits outside the size
    }

    binary_set::binary_set(const binary_set& other_set) {
        this -> capacity_ = other_set.capacity_;
        this -> set_ = std::vector<char>(other_set.set_);
    }

    void binary_set::add(size_t element) {

        #if HPLUS_INTCHECK
        assert(element < this -> capacity_, "binary_set::add failed.");
        #endif
        this -> set_[element/8] |= (1 << element%8);

    }

    void binary_set::remove(size_t element) {

        #if HPLUS_INTCHECK
        assert(element < this -> capacity_, "binary_set::remove failed.");
        #endif
        this -> set_[element/8] &= ~(1 << element%8);

    }

    void binary_set::clear() { this -> set_ = std::vector<char>((this -> capacity_+7)/8, 0); }

    binary_set binary_set::operator&(const binary_set& other_set) const {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set::& failed.");
        #endif
        binary_set new_bitfield(*this);
        for (size_t i = 0; i < this -> set_.size(); i++) new_bitfield.set_[i] &= other_set.set_[i];
        return new_bitfield;

    }

    binary_set& binary_set::operator&=(const binary_set& other_set) {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set::&= failed.");
        #endif
        for (size_t i = 0; i < this -> set_.size(); i++) this -> set_[i] &= other_set.set_[i];
        return *this;

    }

    binary_set binary_set::operator|(const binary_set& other_set) const {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set::| failed.");
        #endif
        binary_set new_bitfield(*this);
        for (size_t i = 0; i < this -> set_.size(); i++) new_bitfield.set_[i] |= other_set.set_[i];
        return new_bitfield;

    }

    binary_set& binary_set::operator|=(const binary_set& other_set) {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set::|= failed.");
        #endif
        for (size_t i = 0; i < this -> set_.size(); i++) this -> set_[i] |= other_set.set_[i];
        return *this;

    }

    binary_set binary_set::operator-(const binary_set& other_set) const {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set::- failed.");
        #endif
        binary_set new_bitfield(*this);
        for (size_t i = 0; i < this -> set_.size(); i++) new_bitfield.set_[i] &= ~other_set.set_[i];
        return new_bitfield;

    }

    binary_set& binary_set::operator-=(const binary_set& other_set) {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set::-= failed.");
        #endif
        for (size_t i = 0; i < this -> set_.size(); i++) this -> set_[i] &= ~other_set.set_[i];
        return *this;

    }

    binary_set binary_set::operator!() const {

        binary_set new_bitfield(this -> capacity_);
        for (size_t i = 0; i < this -> set_.size(); i++) new_bitfield.set_[i] = ~this -> set_[i];
        if (this -> capacity_ % 8 != 0) new_bitfield.set_[this -> set_.size() - 1] &= (1u << this -> capacity_ % 8) - 1;        // mask for the bits outside the size
        return new_bitfield;

    }

    bool binary_set::operator[](size_t element) const {

        #if HPLUS_INTCHECK
        assert(element < this -> capacity_, "binary_set::[] failed.");
        #endif
        return this -> set_[element/8] & (1 << element%8);

    }

    size_t binary_set::size() const { return this -> capacity_; }

    bool binary_set::operator==(const binary_set& other_set) const {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set:== failed.");
        #endif
        size_t i;
        for (i = 0; i < this -> set_.size() && this -> set_[i] == other_set.set_[i]; i++);
        return i == this -> set_.size();

    }

    bool binary_set::operator!=(const binary_set& other_set) const { return !((*this)==other_set); }

    bool binary_set::intersects(const binary_set& other_set) const {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set:intersects failed.");
        #endif
        size_t i;
        for (i = 0; i < this -> set_.size() && !(this -> set_[i] & other_set.set_[i]); i++);
        return i != this -> set_.size();

    }

    bool binary_set::contains(const binary_set& other_set) const {

        #if HPLUS_INTCHECK
        assert(this -> capacity_ == other_set.capacity_, "binary_set:contains failed.");
        #endif
        size_t i;
        for (i = 0; i < this -> set_.size() && !(~this -> set_[i] & other_set.set_[i]); i++);
        return i == this -> set_.size();

    }

    std::vector<size_t> binary_set::sparse() const {

        std::vector<size_t> ret;
        for (auto p : *this) ret.push_back(p);
        return ret;

    }

    binary_set::operator std::string() const {

        std::string ret = "[";
        for (size_t i = 0; i < this -> capacity_; i++) ret.append(this -> operator[](i) ? "X" : " ");
        ret.append("]");
        return ret;

    }

    binary_set::Iterator::Iterator(const binary_set *bitField, size_t index) {

        set_ = bitField;
        current_element_ = index;
        if (current_element_ < set_->size() && !(*set_)[current_element_]) ++(*this);

    }

    binary_set::Iterator& binary_set::Iterator::operator++()  {

        do {
            ++current_element_;
        } while (current_element_ < set_ -> size() && !(*set_)[current_element_]);

        return *this;

    }

    size_t binary_set::Iterator::operator*() const { return current_element_; }

    bool binary_set::Iterator::operator!=(const Iterator &other) const { return current_element_ != other.current_element_; }

    binary_set::Iterator binary_set::begin() const { return Iterator(this, 0); }

    binary_set::Iterator binary_set::end() const { return Iterator(this, capacity_); }
    
    // ##################################################################### //
    // ########################## SUBSET SEARCHER ########################## //
    // ##################################################################### //

    subset_searcher::subset_searcher() { root = new treenode(); }

    void subset_searcher::add(size_t value, const binary_set& set) {
        treenode* leaf = this -> root;
        for (size_t i = 0; i < set.size(); i++) {
            if (set[i]) {
                if (leaf -> right == nullptr) leaf -> right = new treenode();
                leaf = leaf -> right;
            } else {
                if (leaf -> left == nullptr) leaf -> left = new treenode();
                leaf = leaf -> left;
            }
        }
        (leaf -> values).push_back(value);
    }

    std::vector<size_t> subset_searcher::find_subsets(const binary_set& set) {
        std::deque<treenode*> open_nodes;
        open_nodes.push_back(this -> root);
        for (size_t i = 0; i < set.size() && !open_nodes.empty(); i++) {
            size_t tmp = open_nodes.size();
            for (size_t _ = 0; _ < tmp; _++) {
                treenode* node = open_nodes.front();
                open_nodes.pop_front();
                if (set[i]) {
                    if (node -> left != nullptr) open_nodes.push_back(node -> left);
                    if (node -> right != nullptr) open_nodes.push_back(node -> right);
                } else {
                    if (node -> left != nullptr) open_nodes.push_back(node -> left);
                }
            }
        }
        std::vector<size_t> result;
        for (auto node : open_nodes) result.insert(result.end(), node -> values.begin(), node -> values.end());
        return result;
    }

    subset_searcher::~subset_searcher() {
        std::deque<treenode*> remaining_nodes;
        remaining_nodes.push_back(this -> root);
        while (!remaining_nodes.empty()) {
            treenode* node = remaining_nodes.front();
            remaining_nodes.pop_front();
            if (node -> left != nullptr) remaining_nodes.push_back(node -> left);
            if (node -> right != nullptr) remaining_nodes.push_back(node -> right);
            delete node;
        }
    }

    // ##################################################################### //
    // ############################### LOGGER ############################## //
    // ##################################################################### //

    void logger::format_output_(const char* str, va_list ptr, FILE* log_file, bool print_log, bool show_time, bool error) {

        if (show_time) {
            double elapsed_time = timer.get_time();
            fprintf(error ? stderr : stdout, "%8.3f : ", elapsed_time);
            if (print_log) fprintf(log_file, "%8.3f : ", elapsed_time);
        }

        // char array to store token
        char token[1000];
        // index of where to store the characters of str in token
        int k = 0;

        // parsing the formatted string
        for (int i = 0; str[i] != '\0'; i++) {

            token[k++] = str[i];
            
            // Check if the next character forms %%
            if (str[i] == '%' && str[i + 1] == '%') {
                // Print a single '%' and skip the second '%'
                fprintf(error ? stderr : stdout, "%%");
                if (print_log) fprintf(log_file, "%%");
                i++; // Skip the second '%'
                k = 0; // Reset token index
                continue;
            }

            if (str[i + 1] == '%' || str[i + 1] == '\0') {

                token[k] = '\0';
                k = 0;
                if (token[0] != '%') {

                    fprintf(error ? stderr : stdout, "%s", token); // printing the whole token if it is not a format specifier
                    if (print_log) fprintf(log_file, "%s", token);

                } else {

                    int j = 1;
                    char ch1;

                    // this loop is required when printing formatted value like 0.2f, when ch1='f' loop ends
                    while ((ch1 = token[j++]) < 58) {}

                    if (ch1 == 'i' || ch1 == 'd' || ch1 == 'u' || ch1 == 'h') { // for integers

                        int value = va_arg(ptr, int);
                        fprintf(error ? stderr : stdout, token, value);
                        if (print_log) fprintf(log_file, token, value);

                    } else if (ch1 == 'c') { // for chars

                        char value = va_arg(ptr, int);
                        fprintf(error ? stderr : stdout, token, value);
                        if (print_log) fprintf(log_file, token, value);

                    } else if (ch1 == 'f') { // for float values

                        double value = va_arg(ptr, double);
                        fprintf(error ? stderr : stdout, token, value);
                        if (print_log) fprintf(log_file, token, value);

                    } else if (ch1 == 'l') { // for long values

                        char ch2 = token[2];

                        if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long int values

                            long value = va_arg(ptr, long);
                            fprintf(error ? stderr : stdout, token, value);
                            if (print_log) fprintf(log_file, token, value);

                        } else if (ch2 == 'f') {  // for double values

                            double value = va_arg(ptr, double);
                            fprintf(error ? stderr : stdout, token, value);
                            if (print_log) fprintf(log_file, token, value);

                        }

                    } else if (ch1 == 'L') {

                        char ch2 = token[2];

                        if (ch2 == 'u' || ch2 == 'd' || ch2 == 'i') { // for long long int values

                            long long value = va_arg(ptr, long long);
                            fprintf(error ? stderr : stdout, token, value);
                            if (print_log) fprintf(log_file, token, value);

                        }

                        else if (ch2 == 'f') { // for long double values

                            long double value = va_arg(ptr, long double);
                            fprintf(error ? stderr : stdout, token, value);
                            if (print_log) fprintf(log_file, token, value);

                        }

                    } else if (ch1 == 's') { // for string values

                        char* value = va_arg(ptr, char*);
                        fprintf(error ? stderr : stdout, token, value);
                        if (print_log) fprintf(log_file, token, value);

                    } else { // print the whole token if no case is matched

                        fprintf(error ? stderr : stdout, "%s", token);
                        if (print_log) fprintf(log_file, "%s", token);

                    }

                }

            }

        }
    }

    logger::logger(const std::string& run_title, bool log_enabled, const std::string& log_name) {

        this -> log_file_ = log_name;
        if (log_enabled)  {
            this -> log_enabled_ = log_enabled;
            FILE* log_file = fopen(this -> log_file_.c_str(), "a");
            fprintf(log_file, "\n%s\nRUN_NAME: %s\n%s\n", THICK_LINE, run_title.c_str(), THICK_LINE);
            fclose(log_file);
        }

    }

    void logger::print(const char* str, ...) const {

        #if HPLUS_VERBOSE == 0
        return;
        #endif

        // logging file
        FILE* log_file = nullptr;
        if (this -> log_enabled_) log_file = fopen(this -> log_file_.c_str(), "a");

        // initializing list pointer
        va_list ptr;
        va_start(ptr, str);

        // printing and logging the formatted message
        format_output_(str, ptr, log_file, this -> log_enabled_, false);

        fprintf(stdout, "\n");
        if (this -> log_enabled_) fprintf(log_file, "\n");

        if (this -> log_enabled_) fclose(log_file);

        // ending traversal
        va_end(ptr);

    }

    void logger::print_info(const char* str, ...) const {

        #if HPLUS_VERBOSE < 10
        return;
        #endif

        // logging file
        FILE* log_file = nullptr;
        if (this -> log_enabled_) log_file = fopen(this -> log_file_.c_str(), "a");

        // initializing list pointer
        va_list ptr;
        va_start(ptr, str);

        fprintf(stdout, "\033[92m\033[1m[ INFO  ]\033[0m -- ");
        if (this -> log_enabled_) fprintf(log_file, "[ INFO  ] -- ");

        // printing and logging the formatted message
        format_output_(str, ptr, log_file, this -> log_enabled_);

        fprintf(stdout, "\n");
        if (this -> log_enabled_) fprintf(log_file, "\n");

        if (this -> log_enabled_) fclose(log_file);

        // ending traversal
        va_end(ptr);

    }

    void logger::print_warn(const char* str, ...) const {

        #if HPLUS_WARN == 0
        return;
        #endif

        // logging file
        FILE* log_file = nullptr;
        if (this -> log_enabled_) log_file = fopen(this -> log_file_.c_str(), "a");

        // initializing list pointer
        va_list ptr;
        va_start(ptr, str);

        fprintf(stdout, "\033[93m\033[1m[ WARN  ]\033[0m -- ");
        if (this -> log_enabled_) fprintf(log_file, "[ WARN  ] -- ");

        // printing and logging the formatted message
        format_output_(str, ptr, log_file, this -> log_enabled_);

        fprintf(stdout, "\n");
        if (this -> log_enabled_) fprintf(log_file, "\n");

        if (this -> log_enabled_) fclose(log_file);

        // ending traversal
        va_end(ptr);

    }

    void logger::raise_error(const char* str, ...) const {

        // logging file
        FILE* log_file = nullptr;
        if (this -> log_enabled_) log_file = fopen(this -> log_file_.c_str(), "a");

        // initializing list pointer
        va_list ptr;
        va_start(ptr, str);

        fprintf(stderr, "\033[91m\033[1m[ ERROR ]\033[0m -- ");
        if (this -> log_enabled_) fprintf(log_file, "[ ERROR ] -- ");

        // printing and logging the formatted message
        format_output_(str, ptr, log_file, this -> log_enabled_, true, true);

        fprintf(stderr, "\n");
        if (this -> log_enabled_) fprintf(log_file, "\n");

        if (this -> log_enabled_) fclose(log_file);

        // ending traversal
        va_end(ptr);

        exit(1);

    }

    // ##################################################################### //
    // ############################ TIME KEEPER ############################ //
    // ##################################################################### //

    void time_keeper::start_timer() { this -> timer = std::chrono::steady_clock::now(); }

    double time_keeper::get_time() const { return ((double) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this -> timer).count()) / 1000; }

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

    void assert(bool condition, const std::string& message) { if (!condition) mylog.raise_error("%s - Assert check failed.", message.c_str()); }

    bool isint(const std::string& str, const int from, const int to) {

        try {
            int num = stoi(str);
            return num >= from && num <= to;
        } catch (std::invalid_argument&) { return false; }

    }

    void todo(const std::string& msg) { mylog.raise_error("%s: UNIMPLEMENTED.", msg.c_str()); }

    void pause(const std::string message) {

        unsigned int test = 0;
        std::cout << message << "(1 to exit, 0 to continue): ";
        std::cin >> test;
        if (test > 0) exit(1);

    }

}