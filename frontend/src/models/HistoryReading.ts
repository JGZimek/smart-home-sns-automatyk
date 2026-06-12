export interface HistoryMeasurement {
  value: number;
  created_at: string;
}

export interface HistoryReading {
  id: string;
  sensor_type: string;
  unit: string;
  room: string;
  measurements: HistoryMeasurement[];
}
