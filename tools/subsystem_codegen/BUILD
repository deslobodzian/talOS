load("@rules_python//python:py_binary.bzl", "py_binary")

py_binary(
    name = "test",
    srcs = ["model.py", "parser.py", "main.py"],
    data = ["test.toml"],
    main = "main.py",
    imports = ["."],
)
