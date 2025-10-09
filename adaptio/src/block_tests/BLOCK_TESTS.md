# Block Tests

## Run

~~~bash
# build
adaptio --build-tests

# run
./build/debug/src/adaptio-block-tests --trace
~~~

### Use filters

~~~bash
# filter test suites
./build/debug/src/adaptio-block-tests --trace --doctest-test-suite=<filters>

# filter test cases
./build/debug/src/adaptio-block-tests --trace --doctest-test-case=<filters>
# e.g
./build/debug/src/adaptio-block-tests --trace --doctest-test-case=myTestCases1*,"TestCase my"
~~~

**Filters:** a comma-separated list of wildcards for matching values - where * means "match any sequence" and ? means "match any one character".<br>
Refer [here](https://github.com/doctest/doctest/blob/master/doc/markdown/commandline.md)

### Break on first failure

~~~bash
# Break on first failure
./build/debug/src/adaptio-block-tests --trace --abort-after=1
# or short
./build/debug/src/adaptio-block-tests --trace -aa=1
~~~

### Persist DB in real time (optional)

By default, block tests use an in-memory SQLite database. To observe and persist writes in real time during tests, set an on-disk path via `ADAPTIO_BLOCK_TEST_DB` before running. WAL mode and FULL synchronous are enabled automatically for durability.

~~~bash
export ADAPTIO_BLOCK_TEST_DB=/tmp/adaptio_block_tests.db
adaptio --build-tests
./build/debug/src/adaptio-block-tests --trace

# Tail SQLite WAL for activity (optional)
ls -l /tmp/adaptio_block_tests.db*
~~~

### Enable traces

Traces are not enabled by default for many block tests, same of them are prepared for tracing by uncomment a line like below.

`// #define TESTCASE_DEBUG_ON 1  // Uncomment for test debug`

Refer [here](basic_abp_welding_test.cc#L36)

## Reference

Refer [here](https://github.com/doctest/doctest/blob/master/doc/markdown/readme.md)
