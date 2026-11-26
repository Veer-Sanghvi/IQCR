const { test, expect } = require("@playwright/test");

async function setSlider(locator, value) {
  await locator.evaluate((el, v) => {
    el.value = String(v);
    el.dispatchEvent(new Event("input", { bubbles: true }));
  }, value);
}

test("auto-demo badge is visible on a fresh load, before any interaction", async ({ page }) => {
  await page.goto("/");
  await expect(page.locator("#auto-demo-badge")).toBeVisible();
});

test.describe("prefers-reduced-motion", () => {
  // page.emulateMedia is used instead of test.use({ reducedMotion }); the latter did not apply in this runner
  test("auto-demo never starts, scene is static, orbit still works", async ({ page }) => {
    const errors = [];
    page.on("pageerror", (err) => errors.push(err.message));
    await page.emulateMedia({ reducedMotion: "reduce" });
    await page.goto("/");
    await expect(page.locator("html")).toHaveAttribute("data-auto-demo", "off");
    const badgeHidden = await page.locator("#auto-demo-badge").evaluate((el) => el.classList.contains("hidden"));
    expect(badgeHidden).toBe(true);
    // wait past the 1400 ms auto-demo kickoff and confirm nothing ran
    await page.waitForTimeout(2500);
    await expect(page.locator("#mode-pill")).toHaveText("MANUAL POSE");
    await expect(page.locator("#live-table tbody tr")).toHaveCount(0);
    // OrbitControls: a drag on the canvas must still work without errors
    const canvas = page.locator("#canvas-host canvas");
    await expect(canvas).toBeVisible();
    const box = await canvas.boundingBox();
    await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
    await page.mouse.down();
    await page.mouse.move(box.x + box.width / 2 + 120, box.y + box.height / 2 + 40, { steps: 8 });
    await page.mouse.up();
    expect(errors).toEqual([]);
  });
});

test.describe("IQCR robot-arm simulation", () => {
  test.beforeEach(async ({ page }) => {
    await page.goto("/");
    // clicking a tab is a pointerdown inside #viewer, which stops the auto-demo
    // so every test after this starts from a deterministic manual-mode state
    await page.locator("#tab-manual").click();
  });

  test("loads with correct title and no console errors", async ({ page }) => {
    const errors = [];
    page.on("pageerror", (err) => errors.push(err.message));
    await page.reload();
    await page.locator("#tab-manual").click();
    await expect(page).toHaveTitle(/Inspection and Quality Control Robots/);
    expect(errors).toEqual([]);
  });

  test("3D canvas renders with nonzero size", async ({ page }) => {
    const canvas = page.locator("#canvas-host canvas");
    await expect(canvas).toBeVisible();
    const box = await canvas.boundingBox();
    expect(box.width).toBeGreaterThan(0);
    expect(box.height).toBeGreaterThan(0);
  });

  test("touching the viewer stops the auto-demo", async ({ page }) => {
    const hidden = await page.locator("#auto-demo-badge").evaluate((el) => el.classList.contains("hidden"));
    expect(hidden).toBe(true);
  });

  test("manual joint slider changes the end-effector readout", async ({ page }) => {
    const before = await page.locator("#ee-x").textContent();
    await setSlider(page.locator("#q0"), 90);
    await expect(page.locator("#v-q0")).toContainText("90");
    await expect(page.locator("#ee-x")).not.toHaveText(before);
  });

  test("reset pose restores the default joint angles", async ({ page }) => {
    await setSlider(page.locator("#q1"), 80);
    await page.locator("#reset-manual").click();
    await expect(page.locator("#v-q1")).toContainText("20");
  });

  test("generating and solving an IK target converges and logs a result", async ({ page }) => {
    await page.locator("#tab-ik").click();
    await page.locator("#gen-target").click();
    await expect(page.locator("#solve-ik")).toBeEnabled();
    await page.locator("#solve-ik").click();
    await expect(page.locator("#ik-result .badge")).toBeVisible({ timeout: 10000 });
    await expect(page.locator("#live-table tbody tr")).toHaveCount(1);
  });

  test("raising the starting-guess error slider updates its label", async ({ page }) => {
    await page.locator("#tab-ik").click();
    await setSlider(page.locator("#perturb"), 40);
    await expect(page.locator("#v-perturb")).toContainText("40");
  });

  test("lowering lambda updates the damping label", async ({ page }) => {
    await page.locator("#tab-ik").click();
    await setSlider(page.locator("#lambda"), -5);
    await expect(page.locator("#v-lambda")).toContainText("1e-5");
  });

  test("workspace envelope chart renders points", async ({ page }) => {
    const count = await page.locator("#chart-envelope circle").count();
    expect(count).toBeGreaterThan(100);
  });

  test("paper's reference results are visible on load with 8 rows each, no expand needed", async ({ page }) => {
    await expect(page.locator("#paper-table tr")).toHaveCount(8);
    await expect(page.locator("#rst-table tr")).toHaveCount(8);
  });
});
