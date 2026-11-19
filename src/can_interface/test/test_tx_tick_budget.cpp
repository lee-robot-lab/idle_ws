#include "can_interface/tx_tick_budget.hpp"

#include <gtest/gtest.h>

TEST(TxTickBudget, AllowsOnlyConfiguredFramesPerTick)
{
  can_interface::TxTickBudget budget(2);

  EXPECT_TRUE(budget.try_consume());
  EXPECT_TRUE(budget.try_consume());
  EXPECT_FALSE(budget.try_consume());
}

TEST(TxTickBudget, ResetStartsNextTick)
{
  can_interface::TxTickBudget budget(1);

  EXPECT_TRUE(budget.try_consume());
  EXPECT_FALSE(budget.try_consume());

  budget.reset();

  EXPECT_TRUE(budget.try_consume());
  EXPECT_FALSE(budget.try_consume());
}
