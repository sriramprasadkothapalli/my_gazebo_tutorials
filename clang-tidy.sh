# Run clang-tidy
clang-tidy $(find src -name "*.cpp") --  -std=c++17 > results/clang_tidy_result.txt
echo "clang-tidy: Done Processing. Results are stored in results/clang_tidy_result.txt"