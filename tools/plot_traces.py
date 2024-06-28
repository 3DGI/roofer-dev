import argparse
import json
from pathlib import Path

import pandas as pd
from matplotlib import pyplot as plt

parser = argparse.ArgumentParser(
    prog='PlotTraces',
    description='Plot the tracing information of roofer')
parser.add_argument('logfile')

if __name__ == "__main__":
    args = parser.parse_args()
    path_log = Path(args.logfile).absolute()

    with path_log.open() as fo:
        log_json = json.load(fo)


    # Parse the trace messages for easy plotting
    def parse_trace_message(log):
        for record in log:
            if record["level"] == "trace":
                if record["message"][0] == "{":
                    m = json.loads(record["message"])
                    del record["message"]
                    del record["name"]
                    del record["level"]
                    record["time"] = pd.to_datetime(record["time"])
                    record.update(m)
                    yield record


    trace_df = pd.DataFrame.from_records(parse_trace_message(log_json["log"]))
    start_time = trace_df.iloc[0]["time"]
    end_time = trace_df["time"].max()
    trace_df.loc[:, "duration"] = trace_df["time"].apply(lambda t: (t - start_time).total_seconds())

    fig, (ax_counts, ax_heap) = plt.subplots(2, 1, layout='constrained')
    # The expected groups are "crop", "reconstruct", "serialize", "heap"
    for name, group_df in trace_df.groupby("name"):
        if name != "heap":
            ax_counts.plot(group_df["duration"], group_df["count"], label=name)
        else:
            ax_heap.plot(group_df["duration"], group_df["count"], label=name)

    ax_counts.set_ylabel("Nr. objects produced")
    ax_counts.legend()
    ax_counts.set_title(f"Total duration {(end_time - start_time).total_seconds():.2f}s")
    ax_heap.set_xlabel(f"Duration of the complete program [s]")
    ax_heap.set_ylabel('Heap allocation [b]')
    plt.savefig(f"roofer_trace_plot.png")
    plt.close()
