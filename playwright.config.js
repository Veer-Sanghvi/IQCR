// @ts-check
const { defineConfig, devices } = require("@playwright/test");

module.exports = defineConfig({
  testDir: "./tests",
  fullyParallel: true,
  reporter: process.env.CI ? [["list"], ["html", { open: "never" }]] : [["list"]],
  use: {
    baseURL: "http://127.0.0.1:4174",
    trace: "retain-on-failure",
    // CI runners have no GPU: WebGL goes through SwiftShader. Force the same
    // path locally with SOFTWARE_GL=1 (CI sets it implicitly) so a run here
    // exercises the low-capability mode the page switches to on such machines.
    launchOptions: process.env.CI || process.env.SOFTWARE_GL
      ? { args: ["--use-gl=angle", "--use-angle=swiftshader", "--disable-gpu"] }
      : {},
  },
  webServer: {
    command: "python3 -m http.server 4174",
    url: "http://127.0.0.1:4174",
    reuseExistingServer: !process.env.CI,
    timeout: 10_000,
  },
  projects: [
    { name: "chromium", use: { ...devices["Desktop Chrome"] } },
  ],
});
