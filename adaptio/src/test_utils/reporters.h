#pragma once

#include <doctest/doctest.h>

#include <boost/log/expressions/attr.hpp>
#include <string>
#include <vector>

#include "testlog.h"

struct ConsoleExtReporter : public doctest::IReporter {
  struct FailedTest {
    std::string file;
    std::string test_suite;
    std::string test_case;
    std::string error_msg;
    std::optional<int> line;
  };
  std::vector<FailedTest> failed_tests;

  const doctest::TestCaseData* tc{};
  explicit ConsoleExtReporter(const doctest::ContextOptions& /*in*/) {}

  void report_query(const doctest::QueryData& /*in*/) override {}

  void test_run_start() override {}

  void test_run_end(const doctest::TestRunStats& /*in*/) override {
    if (!failed_tests.empty()) {
      TESTLOG_NOHDR("\nFailed tests:");

      auto cnt = 0;
      for (auto const& ft : failed_tests) {
        TESTLOG_NOHDR(" {:2}/{}", ++cnt, failed_tests.size());
        TESTLOG_NOHDR("  File: {}{}", ft.file, ft.line ? fmt::format(":{}", ft.line.value()) : "");
        TESTLOG_NOHDR("  TestSuite: {}", ft.test_suite);
        TESTLOG_NOHDR("  TestCase:  {}", ft.test_case);
        TESTLOG_NOHDR("  Msg:       {}\n", ft.error_msg);
      }
    }
  }

  void test_case_start(const doctest::TestCaseData& in) override {
    tc = &in;
    TESTLOG_NOHDR("\n  Starting test: {}::{}", tc->m_test_suite, tc->m_name);
    TESTLOG_NOHDR("  File location: {}", tc->m_file.c_str());
    TESTLOG_NOHDR("  ------------------------------------------------------");
  }

  void test_case_reenter(const doctest::TestCaseData& /*in*/) override {}

  void test_case_end(const doctest::CurrentTestCaseStats& /*in*/) override {}

  void test_case_exception(const doctest::TestCaseException& in) override {
    failed_tests.push_back(FailedTest{
        .file       = tc->m_file.c_str(),
        .test_suite = tc->m_test_suite,
        .test_case  = tc->m_name,
        .error_msg  = in.error_string.c_str(),
    });
  }

  void subcase_start([[maybe_unused]] const doctest::SubcaseSignature& in) override {
    TESTLOG_NOHDR("  Subcase: {}::{}::{}", tc->m_test_suite, tc->m_name, in.m_name.c_str());
  }

  void subcase_end() override {}
  void log_assert(const doctest::AssertData& in) override {
    if (in.m_failed) {
      failed_tests.push_back(FailedTest{
          .file       = in.m_file,
          .test_suite = tc->m_test_suite,
          .test_case  = tc->m_name,
          .error_msg =
              fmt::format("{} {} ( {} )", doctest::failureString(in.m_at), doctest::assertString(in.m_at), in.m_expr),
          .line = in.m_line,
      });
    }
  }

  void log_message(const doctest::MessageData& /*in*/) override {}

  void test_case_skipped(const doctest::TestCaseData& /*in*/) override {}
};
