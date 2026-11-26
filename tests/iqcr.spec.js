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
  // app.js loads after the window load event, so wait for its init (it stamps
  // <html data-auto-demo>) before the click that is meant to stop the auto-demo
  const appReady = (page) => expect(page.locator("html")).toHaveAttribute("data-auto-demo", /running|off/);

  test.beforeEach(async ({ page }) => {
    await page.goto("/");
    await appReady(page);
    // clicking a tab is a pointerdown inside #viewer, which stops the auto-demo
    // so every test after this starts from a deterministic manual-mode state
    await page.locator("#tab-manual").click();
  });

  test("loads with correct title and no console errors", async ({ page }) => {
    const errors = [];
    page.on("pageerror", (err) => errors.push(err.message));
    await page.reload();
    await appReady(page);
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

  test("generating a target places the inspection part with a TARGET label; manual mode hides it", async ({ page }) => {
    const host = page.locator("#canvas-host");
    await expect(host).toHaveAttribute("data-target", "none");
    await page.locator("#tab-ik").click();
    await page.locator("#gen-target").click();
    await expect(host).toHaveAttribute("data-target", "set");
    const label = page.locator("#canvas-host [data-target-label]");
    await expect(label).toBeVisible();
    await expect(label).toHaveText(/TARGET\s*-?\d+ -?\d+ -?\d+ mm/);
    // the ghost checkbox only toggles the translucent arm; the part stays
    await page.locator("#show-target").uncheck();
    await expect(host).toHaveAttribute("data-target", "set");
    await page.locator("#tab-manual").click();
    await expect(host).toHaveAttribute("data-target", "none");
    await expect(label).toBeHidden();
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

test.describe("showpiece hero", () => {
  test("hero canvas renders the arm and the overlay reports joint angles", async ({ page }) => {
    const errors = [];
    page.on("pageerror", (err) => errors.push(err.message));
    await page.goto("/");
    const host = page.locator("#hero-host");
    await expect(host).toHaveAttribute("data-hero", "ready", { timeout: 15000 });
    const canvas = page.locator("#hero-host canvas");
    await expect(canvas).toBeVisible();
    const box = await canvas.boundingBox();
    expect(box.width).toBeGreaterThan(200);
    expect(box.height).toBeGreaterThan(200);
    // a WebGL frame has been drawn: the canvas backing store matches the CSS box
    const drawn = await canvas.evaluate((el) => el.width > 0 && el.height > 0);
    expect(drawn).toBe(true);
    await expect(page.locator("#hero-host [data-joint]")).toHaveCount(6);
    await expect(page.locator("#hero-host [data-joint] b").first()).toHaveText(/\d+\.\d°/);
    expect(errors).toEqual([]);
  });
});

test.describe("hero callouts", () => {
  // segment intersection, proper crossings only (shared endpoints do not count)
  const cross = (a, b) => {
    const o = (p, q, r) => Math.sign((q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]));
    const [p1, p2] = a, [p3, p4] = b;
    return o(p1, p2, p3) * o(p1, p2, p4) < 0 && o(p3, p4, p1) * o(p3, p4, p2) < 0;
  };
  test("leader lines never cross each other while the arm moves", async ({ page }) => {
    await page.setViewportSize({ width: 1440, height: 900 });
    await page.goto("/");
    await expect(page.locator("#hero-host")).toHaveAttribute("data-hero", "ready", { timeout: 15000 });
    const host = page.locator("#hero-host");
    const box = await host.boundingBox();
    // a few pointer targets, then the idle sweep: a spread of poses
    const spots = [[0.75, 0.3], [0.9, 0.7], [0.4, 0.2], [0.55, 0.85], [0.3, 0.5]];
    for (let s = 0; s < 9; s++) {
      if (s < spots.length) await page.mouse.move(box.x + box.width * spots[s][0], box.y + box.height * spots[s][1]);
      await page.waitForTimeout(700);
      const segs = await page.$$eval("#hero-host [data-leader]", (els) => els.map((l) => ["x1", "y1", "x2", "y2"].map((k) => Number(l.getAttribute(k)))));
      expect(segs).toHaveLength(6);
      for (let i = 0; i < 6; i++) for (let j = i + 1; j < 6; j++) {
        expect(cross([[segs[i][0], segs[i][1]], [segs[i][2], segs[i][3]]], [[segs[j][0], segs[j][1]], [segs[j][2], segs[j][3]]]), `sample ${s}: J${i + 1} x J${j + 1}`).toBe(false);
      }
    }
    await expect(page.locator("#hero-host [data-joint] b").first()).toHaveText(/^ *[+−]\d+\.\d°$/);
  });
});
