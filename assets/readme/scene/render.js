// Renders assets/readme/scene/banner.html to assets/readme/banner.png (1280x400) with Playwright.
// Usage, from the repo root: node assets/readme/scene/render.js
// Same local static server as og/render.js: the page imports the repo's own
// kinematics.js, arm-model.js and vendored three.js.
const path = require("path");
const http = require("http");
const fs = require("fs");
const { chromium } = require("playwright");

const ROOT = path.resolve(__dirname, "../../..");
const OUT = path.join(__dirname, "..", "banner.png");
const W = 1280, H = 400;
const MIME = { ".html": "text/html", ".js": "text/javascript", ".woff2": "font/woff2", ".png": "image/png" };

const server = http.createServer((req, res) => {
  const file = path.join(ROOT, decodeURIComponent(req.url.split("?")[0]));
  if (!file.startsWith(ROOT) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) { res.writeHead(404); return res.end(); }
  res.writeHead(200, { "Content-Type": MIME[path.extname(file)] || "application/octet-stream" });
  fs.createReadStream(file).pipe(res);
});

server.listen(0, "127.0.0.1", async () => {
  const port = server.address().port;
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: W, height: H }, deviceScaleFactor: 1 });
  page.on("pageerror", (e) => { console.error(e); process.exitCode = 1; });
  page.on("console", (m) => { if (m.type() === "error" || m.type() === "warning") console.error(m.text()); else console.log(m.text()); });
  await page.goto(`http://127.0.0.1:${port}/assets/readme/scene/banner.html`);
  await page.waitForSelector("body[data-ready]");
  await page.screenshot({ path: OUT, type: "png" });
  await browser.close();
  server.close();
  console.log(`wrote ${OUT} (${(fs.statSync(OUT).size / 1024).toFixed(0)} KB)`);
});
