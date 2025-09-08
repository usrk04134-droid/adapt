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

### Enable traces

Traces are not enabled by default for many block tests, same of them are prepared for tracing by uncomment a line like below.

`// #define TESTCASE_DEBUG_ON 1  // Uncomment for test debug`

Refer [here](basic_abp_welding_test.cc#L36)

## Reference

Refer [here](https://github.com/doctest/doctest/blob/master/doc/markdown/readme.md)
