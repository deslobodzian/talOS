load("@rules_python//python:py_binary.bzl", "py_binary")

py_binary(
    name = "test",
    srcs = ["model.py", "parser.py", "main.py", "generator.py"],
    data = ["test.toml"],
    main = "main.py",
    imports = ["."],
)
