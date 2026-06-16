export const ROUTES = {
  MAIN: "/",
  SENSORS: "/sensors",
  ROOMS: "/rooms",
  SENSOR_HISTORY: (id: string) => `/sensors/${id}/history`,
};
