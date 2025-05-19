import numpy as np

# ----------------------
# Zero-Sum English Auction Simulator with Budget Carry-Over
# ----------------------

# Game parameters
N_ITEMS = 10          # number of items to auction
START_PRICE = 0       # starting bid price
INCREMENT = 1         # price increment per round
BUDGET = 100          # starting budget for each player


def simulate_auction(valuations, budgets=None):
    """
    Simulate an English (ascending) auction for multiple items with carry-over budgets.

    Each item is auctioned sequentially:
      - Budgets roll over between items, so spending on earlier items reduces capacity later.
      - Start at START_PRICE; at each increment, players drop out when price > valuation or budget.
      - Last remaining bidder wins and pays the current price.

    Returns:
        winners: (N_ITEMS,) array of winner indices (0=A,1=B)
        prices: (N_ITEMS,) array of final sale prices
        budgets_history: list of (before_budget_A, before_budget_B, after_budget_A, after_budget_B)
        history: detailed bidding history for each item
    """
    n_items = valuations.shape[1]
    budgets = np.array([BUDGET, BUDGET]) if budgets is None else budgets.copy()

    winners = np.full(n_items, -1, dtype=int)
    prices = np.zeros(n_items, dtype=int)
    history = []
    budgets_history = []

    for i in range(n_items):
        # Record budgets before this item's auction
        before_A, before_B = budgets.copy()
        budgets_history.append([before_A, before_B, None, None])

        price = START_PRICE
        active = [True, True]
        item_hist = []

        # Ascending bidding
        while sum(active) > 1:
            item_hist.append((price, active.copy()))
            price += INCREMENT
            for p in (0, 1):
                if not active[p]:
                    continue
                if price > valuations[p, i] or price > budgets[p]:
                    active[p] = False

                # Determine winner and final price
        # The auction ends when price has been incremented to a level where a player drops out.
        # That price is one unit above the loser’s valuation or budget, so use it as the sale price.
        final_price = price
        prices[i] = final_price
        if sum(active) == 1:
            winner = 0 if active[0] else 1
        else:
            # if both drop simultaneously, random tie-break
            winner = np.random.choice([0, 1])([0, 1])
        winners[i] = winner

        # Deduct budget and record after budgets
        budgets[winner] -= final_price
        after_A, after_B = budgets.copy()
        budgets_history[-1][2] = after_A
        budgets_history[-1][3] = after_B

        # Append final state to history
        item_hist.append((final_price, active.copy()))
        history.append(item_hist)

    return winners, prices, budgets_history, history


def main():
    # Initialize private valuations
    valuations = np.random.randint(0, BUDGET // 2 + 1, size=(2, N_ITEMS))

    # Run auction with carry-over budgets
    winners, prices, budgets_history, history = simulate_auction(valuations)

    # Compute utilities for each player
    utilities = np.zeros(2)
    for i, w in enumerate(winners):
        utilities[w] += valuations[w, i] - prices[i]

    # Print summary results
    print("Private valuations A:   ", valuations[0])
    print("Private valuations B:   ", valuations[1])
    print("Winners per item:       ", winners)
    print("Sale prices per item:   ", prices)
    print("Utilities (value-price): A={:.1f}, B={:.1f}".format(utilities[0], utilities[1]))
    print("Zero-sum payoff U^A:    {:.1f}\n".format(utilities[0] - utilities[1]))

    # Show budget carry-over details
    print("\nBudget carry-over per item:")
    print("Item | Before A | Before B | After A | After B")
    for idx, (bA0, bB0, bA1, bB1) in enumerate(budgets_history):
        print(f"{idx:>4} | {bA0:>8} | {bB0:>8} | {bA1:>7} | {bB1:>7}")

    # Example: bidding history for item 0
    print("\nBidding history for item 0:")
    for price, active in history[0]:
        print(f" Price={price}, Active players={active}")

if __name__ == '__main__':
    main()
