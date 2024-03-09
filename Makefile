clean:
	find . -name '*.pyc' -exec rm --force {} +
	find . -name '*.pyo' -exec rm --force {} +

delint: format lint

format:
	yapf3 -i -r --style .style.yapf --no-local-style .

lint:
	find . -path ./depricated -prune -o -name '*.py' -exec pylint --rcfile=pylintrc {} +

.PHONY: test
test:
	nosetests3 student_function_unittest.py
