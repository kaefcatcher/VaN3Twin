import unittest
import pandas as pd
from msglog_analyzer import MsgLogAnalyzer

class TestMsgLogAnalyzer(unittest.TestCase):

    def setUp(self):
        data = {
            "senderStationId": [1, 2, 1],
            "receiverStationId": [0, 0, 0],
            "messageType": ["CAM", "CAM", "DENM"],
            "decoded": ["Successfully", "Failed", "Successfully"],
            "distance_m": [100, 200, 150],
            "harm": [1.0, 2.0, 1.5],
            "lossType": ["N/A", "collision", None]
        }

        self.df = pd.DataFrame(data)

        self.analyzer = MsgLogAnalyzer()
        self.analyzer.df = self.df
        self.analyzer.preprocess()

    def test_status_conversion(self):
        statuses = self.analyzer.df["status"].tolist()
        self.assertIn("Accepted", statuses)
        self.assertIn("Rejected", statuses)

    def test_metrics(self):
        df = self.analyzer.df

        total = len(df)
        accepted = len(df[df["status"] == "Accepted"])

        self.assertEqual(total, 3)
        self.assertEqual(accepted, 2)

    def test_plot_data_exists(self):
        data = self.analyzer.get_plot_data()

        self.assertIn("status_counts", data)
        self.assertIn("harm", data)
        self.assertIn("distance", data)
        self.assertIn("loss_type", data)

    def test_loss_type_filled(self):
        loss_counts = self.analyzer.get_plot_data()["loss_type"]
        self.assertIn("No loss", loss_counts.index)

if __name__ == "__main__":
    unittest.main()
