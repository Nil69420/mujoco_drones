## What and why

<!-- What changes, and the problem it solves. -->

## How it was verified

- [ ] New or changed behaviour has a test, and the test fails when the behaviour is broken
- [ ] `ctest` passes locally, and the live and end-to-end tests that apply were run
- [ ] `cmake --build build --target lint` and cppcheck are clean
- [ ] Line coverage stays above the 80 percent gate (the coverage job reports it)
- [ ] Sanitizer, valgrind or fuzz runs were done for changes to memory handling or parsers

## Notes for the reviewer

- [ ] Documentation under `docs/` is updated
- [ ] No new dependency, or it is pinned and justified here
- [ ] The commit messages follow `<type>(<scope>): <summary>`
