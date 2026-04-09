import pandas as pd
import glob
import matplotlib.pyplot as plt


class MsgLogAnalyzer:
    def __init__(self, path="."):
        self.path = path
        self.df = None

    def load_logs(self):
        files = glob.glob(f"{self.path}/*MSGLOG.csv")

        if not files:
            raise FileNotFoundError("MSGLOG files not found")

        df_list = []
        for f in files:
            df = pd.read_csv(f)
            df_list.append(df)

        self.df = pd.concat(df_list, ignore_index=True)
        print(f"Loaded {len(self.df)} records from {len(files)} files")

    def preprocess(self):
        df = self.df

        df = df.rename(columns={
            "senderStationId": "vehicle_1",
            "receiverStationId": "vehicle_2",
            "messageType": "message_type",
            "decoded": "status",
            "distance_m": "distance",
            "lossType": "loss_type"
        })

        df["status"] = df["status"].apply(
            lambda x: "Accepted" if x == "Successfully" else "Rejected"
        )

        self.df = df

    def compute_metrics(self):
        df = self.df

        print("\n=== Metrics ===")

        total = len(df)
        success = len(df[df["status"] == "Accepted"])

        prr = success / total if total > 0 else 0

        print(f"Total messages: {total}")
        print(f"Accepted: {success}")
        print(f"PRR: {prr:.3f}")
        print(f"Average harm: {df['harm'].mean():.3f}")
        print(f"Average distance: {df['distance'].mean():.2f}")

    def save_to_csv(self):
        output_file = "analyzed_msglog.csv"

        df_out = self.df[[
            "vehicle_1",
            "vehicle_2",
            "message_type",
            "status",
            "distance",
            "loss_type",
            "harm"
        ]]

        df_out.to_csv(output_file, index=False)

        print(f"\nCSV saved to {output_file}")

    def visualize(self):
        df = self.df

        # Distance vs Reception
        plt.figure()
        accepted = df[df["status"] == "Accepted"]
        rejected = df[df["status"] == "Rejected"]

        plt.scatter(accepted["distance"], [1]*len(accepted), label="Accepted")
        plt.scatter(rejected["distance"], [0]*len(rejected), label="Rejected")

        plt.xlabel("Distance (m)")
        plt.ylabel("Reception Status")
        plt.title("Distance vs Message Reception")
        plt.legend()
        plt.grid()

        # Harm vs Distance
        plt.figure()
        plt.scatter(df["distance"], df["harm"])
        plt.xlabel("Distance (m)")
        plt.ylabel("Harm")
        plt.title("Harm vs Distance")
        plt.grid()

        # Loss Type
        plt.figure()
        loss_counts = df["loss_type"].fillna("No loss").value_counts()

        if not loss_counts.empty:
            loss_counts.plot(kind="bar")
            plt.title("Loss Type Distribution")
            plt.ylabel("Count")
        else:
            print("No loss_type data to plot")

        plt.show()
    def get_plot_data(self):
        df = self.df

        return {
            "status_counts": df["status"].value_counts(),
            "harm": df["harm"],
            "distance": df["distance"],
            "loss_type": df["loss_type"].fillna("No loss").value_counts()
        }

if __name__ == "__main__":
    analyzer = MsgLogAnalyzer(path=".")

    analyzer.load_logs()
    analyzer.preprocess()
    analyzer.compute_metrics()
    analyzer.save_to_csv()
    analyzer.visualize()
