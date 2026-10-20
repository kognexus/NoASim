import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    include: ["**/src/**/*.test.ts"],
    exclude: ["**/node_modules/**"],
    // Simulation tests run seconds of 4 kHz physics.
    testTimeout: 120_000,
  },
});
