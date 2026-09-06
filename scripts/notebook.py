# /// script
# requires-python = ">=3.14"
# dependencies = [
#     "marimo",
#     "polars",
#     "plotnine",
#     "matplotlib",
#     "scipy",
#     "numpy",
#     "pyarrow==25.0.0",
# ]
# ///

import marimo

__generated_with = "0.23.16"
app = marimo.App(width="medium")


@app.cell
def _():
    import sys
    from pathlib import Path

    import marimo as mo
    import polars as pl

    NB_DIR = Path(mo.notebook_dir() or Path.cwd())
    if str(NB_DIR) not in sys.path:
        sys.path.insert(0, str(NB_DIR))

    import analysis_utils as au
    import barplot as barplot_s
    import boxplot as boxplot_s
    import scatterplot as scatter_s
    import table as table_s

    return NB_DIR, Path, au, barplot_s, boxplot_s, mo, pl, scatter_s, table_s


@app.cell
def _(au, mo):
    mo.md(f"""
    # hplus — run comparison

    Interactive front-end for the analysis scripts
    (`table.py`, `barplot.py`, `boxplot.py`, `scatterplot.py`).

    Every section is a foldable menu with the runs to compare plus all the
    options of the corresponding script; nothing is computed until you press
    its **Run** button. Runs are the CSV files produced by `parse_results.py`;
    the **baseline** is always the first file passed to the script.

    *Constants (from `analysis_utils.py`): time limit `{au.TIME_LIMIT}s`,
    SGM shift `{au.SHIFT}`, significance level `{au.PVALUE}`.*
    """)
    return


@app.cell
def _(NB_DIR, mo):
    dir_input = mo.ui.text(
        value=str(NB_DIR / "results"),
        label="Results directory",
        full_width=True,
    )

    mo.accordion({"### ⚙️ Data source": mo.vstack([dir_input])})
    return (dir_input,)


@app.cell
def _(Path, dir_input, mo, pl):
    runs_dir = Path(dir_input.value).expanduser()
    csv_paths = sorted(runs_dir.glob("*.csv")) if runs_dir.is_dir() else []

    RUNS = {p.stem: str(p) for p in csv_paths}

    _cols: list[str] = []
    for _p in csv_paths:
        for _c in pl.read_csv(_p, n_rows=0).columns:
            if _c not in _cols:
                _cols.append(_c)

    # column names as seen by analysis_utils (Instance -> Problem, N_Nodes -> Nodes)
    METRICS = ["Nodes", "Time"] + [
        c for c in _cols if c not in ("Instance", "Status", "N_Nodes", "Time")
    ]
    BOUND_COLS = [c for c in _cols if c.endswith("_LB") or c.endswith("_UB")]

    mo.md(
        f"**{len(RUNS)}** run(s) found in `{runs_dir}`."
        if RUNS
        else f"⚠️ No CSV file in `{runs_dir}` — fix the data source above."
    )
    return BOUND_COLS, METRICS, RUNS


@app.cell
def _(RUNS, au, mo, pl):
    DEFAULT_METRICS = "Nodes + Time (default)"

    # --- section building blocks -------------------------------------------

    def picker_fields(baseline=True):
        """Run-selection fields shared by every section."""
        opts = list(RUNS)
        fields = {}
        if baseline:
            fields["base"] = mo.ui.dropdown(
                options=opts,
                value=opts[0] if opts else None,
                label="**Baseline**",
                searchable=True,
            )
        fields["others"] = mo.ui.multiselect(
            options=opts,
            label="**Compare with**" if baseline else "**Runs**",
            full_width=True,
        )
        fields["aliases"] = mo.ui.text(
            placeholder="alias1, alias2, ... (one per run, baseline first)",
            label="Aliases",
            full_width=True,
        )
        return fields

    def picker_md(baseline=True):
        return ("{base}\n\n" if baseline else "") + "{others}\n\n{aliases}"

    def section(title, template, fields, button="Run"):
        """A foldable menu whose values reach the compute cell only on submit."""
        form = mo.md(template).batch(**fields).form(
            submit_button_label=button, bordered=False
        )
        return form, mo.accordion({title: form})

    # --- option decoding ----------------------------------------------------

    def resolve(cfg):
        """(files, aliases) in script order: baseline first, then the others."""
        base = cfg.get("base")
        names = ([base] if base else []) + [n for n in cfg["others"] if n != base]
        files = [RUNS[n] for n in names]
        labels = [a.strip() for a in cfg["aliases"].split(",") if a.strip()]
        return files, (labels if len(labels) == len(names) else names)

    def opt_time(value):
        """Time cutoff: 0 means no cutoff (script default: --time unset)."""
        return float(value) if value else None

    def extra_cols(metric):
        """Metric columns other than Nodes/Time must be requested explicitly."""
        return [metric] if metric not in ("Nodes", "Time") else None

    def render(plot, size=None, out=""):
        """Draw a plotnine plot for display, optionally saving it like save()."""
        fig = plot.draw()
        if size:
            fig.set_size_inches(*size)
        if out.strip():
            fig.savefig(out.strip(), bbox_inches="tight")
        return fig

    def waiting(msg="Set the options above and press **Run**."):
        return mo.md(msg).callout("neutral")

    def top_differences(data, names, metric, mode, top_n, shift=0.0):
        """Instances whose metric differs most from the baseline.

        `mode`: *absolute* (largest gap either way), *higher* (metric grew the
        most), *lower* (metric dropped the most). The ratio is
        `(compared + shift) / (baseline + shift)`, in percent: 100% means no
        change. A larger `shift` flattens the ratio of small-metric (easy)
        instances, so the ranking is dominated by the hard ones; `shift = 0`
        is the plain ratio, where easy instances swing the most.
        """
        shift = float(shift)
        base = names[0]
        data = au.require_complete(data, [metric])

        ref = data.filter(pl.col("Model") == base).select(
            "Problem", pl.col(metric).cast(pl.Float64).alias("_ref")
        )
        frames = [
            ref.join(
                data.filter(pl.col("Model") == model).select(
                    "Problem", pl.col(metric).cast(pl.Float64).alias("_cmp")
                ),
                on="Problem",
            ).with_columns(pl.lit(model).alias("Model"))
            for model in names[1:]
        ]
        if not frames:
            return None

        df = pl.concat(frames).with_columns(
            pl.when((pl.col("_ref") + shift == 0) & (pl.col("_cmp") + shift == 0))
            .then(pl.lit(100.0))  # 0 vs 0: identical, no change
            .otherwise((pl.col("_cmp") + shift) / (pl.col("_ref") + shift) * 100)
            .round(2)
            .alias("Ratio_%")
        )
        key = {
            "absolute": (pl.col("Ratio_%") - 100).abs(),
            "higher": pl.col("Ratio_%"),
            "lower": -pl.col("Ratio_%"),
        }[mode]
        return (
            df.sort(key, descending=True)
            .head(int(top_n))
            .select(
                "Problem",
                "Model",
                pl.col("_ref").alias(f"{metric}({base})"),
                pl.col("_cmp").alias(f"{metric}(compared)"),
                "Ratio_%",
            )
        )

    return (
        DEFAULT_METRICS,
        extra_cols,
        opt_time,
        picker_fields,
        picker_md,
        render,
        resolve,
        section,
        top_differences,
        waiting,
    )


@app.cell
def _(DEFAULT_METRICS, METRICS, au, mo, picker_fields, picker_md, section):
    tbl_form, tbl_ui = section(
        "### 📋 Table — `table.py`",
        "Rows: *all* / *solvable* / *all-solvable* + one per time bracket "
        "(or one per family). Columns: solved counts and SGM ratios vs the "
        "baseline; `*` marks p < PVALUE.\n\n"
        + picker_md()
        + "\n\n{metric}\n\n{domain}\n\n{time} &emsp; {family}"
        + "\n\n{diff} &emsp; {top}\n\n{shift}",
        {
            **picker_fields(),
            "metric": mo.ui.dropdown(
                options=[DEFAULT_METRICS] + METRICS,
                value=DEFAULT_METRICS,
                label="`--metric` (custom column, disables significance tests)",
                searchable=True,
            ),
            "domain": mo.ui.text(
                placeholder="substring, e.g. blocks",
                label="`--domain`",
                full_width=True,
            ),
            "time": mo.ui.number(
                start=0,
                stop=float(au.TIME_LIMIT),
                step=1,
                value=0,
                label="`--time` T (0 = off)",
            ),
            "family": mo.ui.checkbox(label="`--by-family`"),
            "diff": mo.ui.dropdown(
                options=["off", "absolute", "lower", "higher"],
                value="off",
                label="Most different instances (metric above, `Time` if default)",
            ),
            "top": mo.ui.number(start=1, stop=200, step=1, value=10, label="rows"),
            "shift": mo.ui.slider(
                steps=[0, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1_000, 10_000, 100_000],
                value=0,
                label="ratio shift (0 = plain ratio, higher = only hard instances)",
                show_value=True,
                full_width=True,
            ),
        },
        button="Run table",
    )
    tbl_ui
    return (tbl_form,)


@app.cell
def _(
    DEFAULT_METRICS,
    au,
    mo,
    opt_time,
    resolve,
    table_s,
    tbl_form,
    top_differences,
    waiting,
):
    mo.stop(tbl_form.value is None, waiting())
    _cfg = tbl_form.value
    mo.stop(_cfg["base"] is None, waiting("Select a baseline run."))

    _files, _names = resolve(_cfg)
    _metric = None if _cfg["metric"] == DEFAULT_METRICS else _cfg["metric"]

    _data = au.prepare_data(
        _files,
        _names,
        domain=_cfg["domain"],
        extra_cols=[_metric] if _metric else None,
        max_time=opt_time(_cfg["time"]),
    )

    _compute = (
        table_s.compute_problem_stats if _cfg["family"] else table_s.compute_stats
    )
    tbl_result = (
        _compute(_data, _names, metrics=(_metric, "Time"), test_significance=False)
        if _metric
        else _compute(_data, _names)
    )

    _mode = _cfg["diff"]
    tbl_diff = (
        top_differences(
            _data, _names, _metric or "Time", _mode, _cfg["top"], _cfg["shift"]
        )
        if _mode != "off"
        else None
    )
    _out = (
        mo.vstack(
            [
                tbl_result,
                mo.md(
                    f"**{_cfg['top']} most different instances** "
                    f"(`{_metric or 'Time'}`, {_mode}) — `Ratio_%` is "
                    f"(compared + {_cfg['shift']}) / (baseline + "
                    f"{_cfg['shift']}) in percent (100% = no change)."
                ),
                tbl_diff,
            ]
        )
        if tbl_diff is not None
        else tbl_result
    )
    _out
    return


@app.cell
def _(METRICS, mo, picker_fields, picker_md, section):
    bar_form, bar_ui = section(
        "### 📊 Bar plot — `barplot.py`",
        "Grouped bars: SGM ratio of the metric vs the baseline, one group per "
        "category / time bracket (`[900,+inf)` omitted), one bar per "
        "non-baseline run.\n\n"
        + picker_md()
        + "\n\n{metric}\n\n{domain}\n\n{out}",
        {
            **picker_fields(),
            "metric": mo.ui.dropdown(
                options=METRICS, value="Time", label="`--metric`", searchable=True
            ),
            "domain": mo.ui.text(
                placeholder="substring, e.g. blocks",
                label="`--domain`",
                full_width=True,
            ),
            "out": mo.ui.text(
                placeholder="bar_plot.pdf (empty = display only)",
                label="`--out`",
                full_width=True,
            ),
        },
        button="Run bar plot",
    )
    bar_ui
    return (bar_form,)


@app.cell
def _(au, bar_form, barplot_s, extra_cols, mo, render, resolve, waiting):
    mo.stop(bar_form.value is None, waiting())
    _cfg = bar_form.value
    mo.stop(
        _cfg["base"] is None or not _cfg["others"],
        waiting("Select a baseline **and** at least one run to compare."),
    )

    _files, _names = resolve(_cfg)
    _metric = _cfg["metric"]

    _data = au.prepare_data(
        _files, _names, domain=_cfg["domain"], extra_cols=extra_cols(_metric)
    )
    _data = au.require_complete(_data, [_metric])

    _plot_df, _order = barplot_s.sgm_ratios(_data, _names, _metric)
    bar_fig = render(
        barplot_s.bar_plot(_plot_df, _order, _names, _metric),
        size=(10, 5),
        out=_cfg["out"],
    )
    bar_fig
    return


@app.cell
def _(BOUND_COLS, au, boxplot_s, mo, picker_fields, picker_md, section):
    box_form, box_ui = section(
        "### 📦 Box plot — `boxplot.py`",
        "Gap (%) between each bound column and the best known incumbent "
        f"(`{boxplot_s.BEST_KNOWN.name}`), one group of boxes per bound, one "
        "box per run. No baseline: all runs are plotted side by side.\n\n"
        + picker_md(baseline=False)
        + "\n\n{gap}\n\n{domain}\n\n{points} &emsp; {solved} &emsp; {time}\n\n{out}",
        {
            **picker_fields(baseline=False),
            "gap": mo.ui.multiselect(
                options=BOUND_COLS,
                value=[
                    c for c in ("Relax_LB", "Root_LB", "Final_LB") if c in BOUND_COLS
                ],
                label="`--gap` bound column(s) — **required**",
                full_width=True,
            ),
            "domain": mo.ui.text(
                placeholder="substring, e.g. blocks",
                label="`--domain`",
                full_width=True,
            ),
            "points": mo.ui.checkbox(label="`--points`"),
            "solved": mo.ui.checkbox(label="`--solved`"),
            "time": mo.ui.number(
                start=0,
                stop=float(au.TIME_LIMIT),
                step=1,
                value=0,
                label="`--time` T (0 = off)",
            ),
            "out": mo.ui.text(
                placeholder="boxplot.pdf (empty = display only)",
                label="`--out`",
                full_width=True,
            ),
        },
        button="Run box plot",
    )
    box_ui
    return (box_form,)


@app.cell
def _(au, box_form, boxplot_s, mo, opt_time, pl, render, resolve, waiting):
    mo.stop(box_form.value is None, waiting())
    _cfg = box_form.value
    mo.stop(
        not _cfg["others"] or not _cfg["gap"],
        waiting("Select at least one run **and** one bound column."),
    )
    mo.stop(
        not boxplot_s.BEST_KNOWN.is_file(),
        mo.md(f"Missing best-known file: `{boxplot_s.BEST_KNOWN}`").callout("danger"),
    )

    _files, _names = resolve(_cfg)
    _gap_cols = list(_cfg["gap"])

    _data = au.prepare_data(
        _files,
        _names,
        domain=_cfg["domain"],
        extra_cols=_gap_cols,
        max_time=opt_time(_cfg["time"]),
    )
    if _cfg["solved"]:
        _data = _data.filter(pl.col("Category") == "all-solvable")

    _gaps = boxplot_s.compute_gaps(_data, _gap_cols)
    # drop problems whose gap is 0 for all models/phases
    _gaps = _gaps.filter(pl.col("Gap").fill_null(0).abs().max().over("Problem") > 0)

    box_fig = render(
        boxplot_s.gap_boxplot(_gaps, _gap_cols, _names, _cfg["points"]),
        size=(max(8, len(_gap_cols) * 1.5 * len(_names)), 5),
        out=_cfg["out"],
    )
    box_fig
    return


@app.cell
def _(METRICS, mo, picker_fields, picker_md, section):
    sct_form, sct_ui = section(
        "### 🔵 Scatter plot — `scatterplot.py`",
        "Log-log scatter of the metric, one facet per non-baseline run vs the "
        "baseline. Reference lines: y=x (black), y=2x / y=x/2 (orange), "
        "y=10x / y=x/10 (red).\n\n"
        + picker_md()
        + "\n\n{metric}\n\n{domain}\n\n{out}",
        {
            **picker_fields(),
            "metric": mo.ui.dropdown(
                options=METRICS, value="Time", label="`--metric`", searchable=True
            ),
            "domain": mo.ui.text(
                placeholder="substring, e.g. blocks",
                label="`--domain` (highlights, does not filter)",
                full_width=True,
            ),
            "out": mo.ui.text(
                placeholder="scatter.pdf (empty = display only)",
                label="`--out`",
                full_width=True,
            ),
        },
        button="Run scatter plot",
    )
    sct_ui
    return (sct_form,)


@app.cell
def _(au, extra_cols, mo, pl, render, resolve, scatter_s, sct_form, waiting):
    mo.stop(sct_form.value is None, waiting())
    _cfg = sct_form.value
    mo.stop(
        _cfg["base"] is None or not _cfg["others"],
        waiting("Select a baseline **and** at least one run to compare."),
    )

    _files, _names = resolve(_cfg)
    _metric = _cfg["metric"]

    _data = au.prepare_data(_files, _names, extra_cols=extra_cols(_metric))
    _highlight = (
        _data.filter(pl.col("Problem").str.contains(_cfg["domain"]))
        if _cfg["domain"]
        else None
    )

    sct_fig = render(
        scatter_s.scatter_plot(_data, _names, _metric, _highlight),
        out=_cfg["out"],
    )
    sct_fig
    return


if __name__ == "__main__":
    app.run()
