# IQCR: Inspection and Quality Control Robots

Live 3D simulation of an ABB IRB 120 six-axis arm and its inverse-kinematics solver, running client-side in the browser.

**[Live demo →](https://veer-sanghvi.github.io/IQCR/)**

## About

IQCR proposes putting a camera, depth sensor, and laser probe on one robot arm instead of running parts between separate inspection stations. Before any of that sensor fusion gets built, the arm has to physically reach the poses the idea requires.

This page runs the same ABB IRB 120 kinematics and the same damped-least-squares inverse-kinematics solver as the paper, live in 3D:

1. **Manual pose control**: drive all 6 joints directly and watch the arm move, built from its actual Denavit–Hartenberg parameters.
2. **Inverse kinematics**: generate a random target pose, perturb the arm's starting guess, and watch the solver converge back, or fail to if you push the perturbation high enough; the revised paper's failure study traces every reproducible failure to a joint-limit-infeasible target, certified by a closed-form solver.
3. **Reachable workspace envelope**: a live 60×60 grid sweep showing where the hand can physically reach.

Every run is compared against the paper's original MATLAB results and an independent cross-check against MATLAB's Robotics System Toolbox, so the hand-coded solver isn't grading its own homework.

## Paper

Co-authored with Tyler Fong and Syed Irtiza Ali Shah at Wentworth Institute of Technology. Accepted to ICACR 2026 (IEEE, EI Compendex/Scopus indexed); presenting October 2026 in Nanjing, China.

Full paper: [`conference_paper/IQCR.pdf`](conference_paper/IQCR.pdf)

The client-side solver in [`kinematics.js`](kinematics.js) is a direct JavaScript port of [`conference_paper/matlab_source/iqcr_kinematics.m`](conference_paper/matlab_source/iqcr_kinematics.m).

## Repo structure

```
index.html                              Page markup and styling
kinematics.js                           Client-side forward/inverse kinematics solver
scene.js / app.js                       3D viewer and UI wiring
tests/iqcr.spec.js                      Playwright tests
conference_paper/
  IQCR.pdf                              Full conference paper
  matlab_source/iqcr_kinematics.m       Original hand-coded MATLAB solver
  matlab_source/rst_verification.m      Independent cross-check via Robotics System Toolbox
```

## Running locally

The page uses ES module import maps, so it needs to be served rather than opened directly as a file:

```sh
npx serve .
# or
python3 -m http.server
```

Then open the printed local URL in a browser.

## Tests

```sh
npm install
npx playwright install --with-deps chromium
npm test
```

Tests run automatically on push/PR via GitHub Actions ([`.github/workflows/tests.yml`](.github/workflows/tests.yml)).

## Contact

Veer Sanghvi · [veer-sanghvi.github.io](https://veer-sanghvi.github.io/)
