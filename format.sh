git ls-tree --full-tree -r --name-only HEAD . | grep ".*\(\.cc\|\.h\|\.hpp\|\.cpp\|\.cu\)$" | xargs clang-format -i
python -m black .
python -m isort --profile black .
