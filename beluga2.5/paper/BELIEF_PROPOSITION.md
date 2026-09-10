# A simple justification for verification beyond MAP

Let `H_1,...,H_M` be the retained trajectory hypotheses, with weights
`w_h >= 0` and `sum_h w_h = 1`. For a fixed loop candidate `c`, let
`e_h(c) in [0,1]` be its trajectory-prior compatibility under hypothesis `H_h`.
These are the bounded scores produced by the verifier, not assumed probabilities.
Let `m` be a maximum-weight hypothesis, using a fixed tie rule, and define

\[
S(c)=\sum_{h=1}^M w_h e_h(c).
\]

A threshold verifier accepts when `S(c) >= tau`, for a fixed `tau in (0,1)`.
A MAP-only summary retains the MAP trajectory, its weight and its compatibility,
but discards the other trajectories' candidate compatibilities.

**Proposition (MAP information does not generally determine belief verification).**
For every such weighted belief,

\[
w_m e_m(c)\ \leq\ S(c)\ \leq\ w_m e_m(c)+1-w_m,
\qquad |S(c)-e_m(c)|\leq 1-w_m.
\]

Both interval endpoints are attainable by compatibilities in `[0,1]`. If

\[
w_m e_m(c)<\tau\leq w_m e_m(c)+1-w_m,
\]

there are two compatibility assignments with the same weights and MAP
compatibility but different threshold decisions. Consequently, a verifier using
only that MAP summary cannot reproduce the full-belief decision for all such
assignments.

**Proof.** The non-MAP contribution is `sum_{h != m} w_h e_h(c)`. Since each
compatibility lies in `[0,1]`, this contribution lies between zero and
`sum_{h != m} w_h = 1-w_m`. Choosing all non-MAP compatibilities zero or all one
attains these endpoints. Under the stated threshold condition the first choice
rejects and the second accepts, although the MAP summary is unchanged. Finally,
`S-e_m = sum_{h != m} w_h (e_h-e_m)`, whose absolute value is at most `1-w_m`.

**Example matching the central claim.** Two hypotheses have weights `0.6, 0.4`.
The candidate has MAP compatibility zero. With secondary compatibility one,
`S=0.4`; with secondary compatibility zero, `S=0`. At threshold `tau=0.25`, these
lead to acceptance and rejection respectively. A MAP-only score is zero in both.
This demonstrates why a plausible secondary trajectory can change verification.

The result also states a limit: if the MAP compatibility is zero and
`1-w_m < tau`, the remaining modes cannot rescue the loop under this threshold.
Retaining a tiny alternative alone is not sufficient. When all retained
compatibilities are zero, the belief score is zero regardless of their weights.

**What this proves.** Discarding non-MAP candidate compatibility can discard
information needed by the proposed decision rule. The bound also quantifies
when that information can matter.

**What it does not prove.** It does not prove that this score labels real loops
correctly, that approximate weights are calibrated, or that more hypotheses
always improve mapping. It is an elementary convex-combination result, not a
new theorem to market as the paper's main novelty. The scientific contribution
must be the verifier and its experimental behavior.

If instead one establishes calibrated conditional probabilities
`e_h(c)=P(loop valid | H_h, D, c)` and posterior weights
`w_h=P(H_h | D,c)`, the law of total probability gives
`S(c)=P(loop valid | D,c)`. The current deformation scores and branch factors do
not establish those calibration assumptions; the paper should not claim them.

Suggested central claim:

> A MAP trajectory can discard loop-compatibility information carried by
> secondary trajectory modes. We therefore evaluate each loop candidate against
> every retained trajectory hypothesis and aggregate compatibility using the
> current hypothesis weights. We test when this improves loop verification at
> a fixed particle and hypothesis budget.
