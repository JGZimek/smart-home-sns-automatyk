import { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
import { CircularProgress, Typography } from "@mui/material";
import { ScatterChart } from "@mui/x-charts/ScatterChart";
import { BackButton } from "../components/BackButton";
import { ROUTES } from "../Routes";
import { type HistoryReading } from "../models/HistoryReading";

function SensorHistoryChart({ id }: { id: string }) {
  const [history, setHistory] = useState<HistoryReading | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;

    fetch(`http://localhost:3000/history-readings/${id}`)
      .then((res) => {
        if (!res.ok) {
          throw new Error(`Failed to load history (HTTP ${res.status})`);
        }
        return res.json();
      })
      .then((json: HistoryReading) => {
        if (!cancelled) {
          setHistory(json);
        }
      })
      .catch((err: Error) => {
        if (!cancelled) {
          setError(err.message);
          setHistory(null);
        }
      })
      .finally(() => {
        if (!cancelled) {
          setLoading(false);
        }
      });

    return () => {
      cancelled = true;
    };
  }, [id]);

  if (loading) {
    return (
      <>
        <BackButton destination={ROUTES.SENSORS} />
        <CircularProgress />
      </>
    );
  }

  if (error) {
    return (
      <>
        <BackButton destination={ROUTES.SENSORS} />
        <Typography color="error">{error}</Typography>
      </>
    );
  }

  if (!history || history.measurements.length === 0) {
    return (
      <>
        <BackButton destination={ROUTES.SENSORS} />
        <Typography>No historical measurements found.</Typography>
      </>
    );
  }

  const points = history.measurements.map((m, index) => ({
    x: new Date(m.created_at).getTime(),
    y: m.value,
    id: index,
  }));

  return (
    <>
      <BackButton destination={ROUTES.SENSORS} />
      <Typography variant="h5" gutterBottom>
        {history.sensor_type} — {history.room}
      </Typography>
      <Typography variant="body2" color="text.secondary" gutterBottom>
        Unit: {history.unit}
      </Typography>
      <ScatterChart
        height={400}
        grid={{ vertical: true, horizontal: true }}
        xAxis={[
          {
            label: "Time",
            valueFormatter: (ms: number) => new Date(ms).toLocaleString(),
          },
        ]}
        yAxis={[{ label: history.unit }]}
        series={[
          {
            label: history.sensor_type,
            data: points,
          },
        ]}
      />
    </>
  );
}

export default function SensorHistory() {
  const { id } = useParams<{ id: string }>();

  if (!id) {
    return (
      <>
        <BackButton destination={ROUTES.SENSORS} />
        <Typography color="error">Missing sensor id.</Typography>
      </>
    );
  }

  return <SensorHistoryChart id={id} />;
}
