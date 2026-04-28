#!/usr/bin/env python

import numpy as np
import scipy.stats


def wilcoxon_test(x, y, shift=0.0, minAbs=0.05, minSamples=25):
    """Wilcoxon paired signed-rank test on shifted ratios. Returns two-sided p-value."""
    assert len(x) == len(y) and len(x) > 0

    delta = np.around(np.log((y + shift) / (x + shift)), 3)
    delta[np.abs(delta) <= minAbs] = 0.0

    nDiff = np.sum(np.abs(delta) > minAbs)
    nSmaller = np.sum(delta < -minAbs)
    nLarger = np.sum(delta > minAbs)

    if nDiff == 0:
        return 1.0
    if nDiff < minSamples:
        return scipy.stats.binomtest(nSmaller, nSmaller + nLarger, p=0.5).pvalue
    return scipy.stats.wilcoxon(delta[delta != 0.0], zero_method="wilcox")[1]


def mcnemar_test(x, y):
    """McNemar's test on two boolean arrays. Returns two-sided p-value."""
    not_x, not_y = np.logical_not(x), np.logical_not(y)
    n01 = np.logical_and(not_x, y).sum()
    n10 = np.logical_and(x, not_y).sum()
    if n01 + n10 == 0:
        return 1.0
    stat = np.square(np.abs(n01 - n10) - 1) / float(n01 + n10)
    return scipy.stats.chi2(1).sf(stat)
