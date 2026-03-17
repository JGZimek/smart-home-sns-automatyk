import { Card, Group, Paper, Text } from '@mantine/core';

interface SensorCardProps {
  quantity: string;
  value: number;
  unit: string;
}

export function SensorCard({quantity, value, unit}: SensorCardProps) {
  return (
  <Card shadow="xs" p="xl">
    <Text>{quantity}: {value}{unit}</Text>
    </Card>
  );
}
