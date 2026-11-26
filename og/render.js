// Renders og/og-scene.html to og/iqcr-og.png (1200x627) with Playwright.
// Usage, from the repo root: node og/render.js
// Needs the repo's own devDependency (@playwright/test brings playwright).
const path = require("path");
const http = require("http");
const fs = require("fs");
const { chromium } = require("playwright");

const ROOT = path.resolve(__dirname, "..");
const OUT = path.join(__dirname, "iqcr-og.png");
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
  const page = await browser.newPage({ viewport: { width: 1200, height: 627 }, deviceScaleFactor: 1 });
  page.on("pageerror", (e) => { console.error(e); process.exitCode = 1; });
  await page.goto(`http://127.0.0.1:${port}/og/og-scene.html`);
  await page.waitForSelector("body[data-ready]");
  await page.screenshot({ path: OUT, type: "png", omitBackground: false });
  await browser.close();
  server.close();
  console.log(`wrote ${OUT} (${(fs.statSync(OUT).size / 1024).toFixed(0)} KB)`);
});
