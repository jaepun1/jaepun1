/*
  เครื่องจ่ายยาจำลอง: ทำตัวเหมือน ESP32-S3 CAM + บอร์ดหลัก
  ใช้ทดสอบเว็บแอปโดยไม่ต้องมีฮาร์ดแวร์

    cd simulator && npm install && npm start
    แล้วในเว็บแอป แท็บ SETTING ใส่ที่อยู่ ws://localhost:81 กด "ทดสอบเชื่อมต่อ"

  ตัวเลือก:
    PORT=8081 npm start          เปลี่ยนพอร์ต
    TAKE_AFTER=10 npm start      หยิบยาอัตโนมัติหลังจ่ายกี่วินาที (0 = ไม่หยิบ ใช้ปุ่ม t แทน)
    REMIND=300                   เปิดเพลงซ้ำทุกกี่วินาทีถ้ายายังอยู่ (เครื่องจริง 300)

  ปุ่มในเทอร์มินัล:  t = หยิบยาออกจากถาด   p = วางยากลับ   q = ออก
*/
const { WebSocketServer } = require("ws");

const PORT = +(process.env.PORT || 81);
const TAKE_AFTER = +(process.env.TAKE_AFTER ?? 10);
const REMIND = +(process.env.REMIND || 300);
const MAX_RINGS = 3;
const MISSED_MS = 60 * 60 * 1000;

const wss = new WebSocketServer({ port: PORT });
let clockOffset = 0;                 // เวลาเครื่อง = เวลาคอม + offset (ตั้งจากคำสั่ง time)
let floors = [];                     // [{date, slots:[{on,h,m,done}]}]  index = floor-1
let cur = { f: 1, s: 1 };
let pill = false;
let dose = null;                     // {f,s,scheduled,start,lastRing,rings,timer}
let busy = Promise.resolve();

const now = () => new Date(Date.now() + clockOffset);
const two = n => String(n).padStart(2, "0");
const isoDate = d => `${d.getFullYear()}-${two(d.getMonth() + 1)}-${two(d.getDate())}`;
const hms = d => `${two(d.getHours())}:${two(d.getMinutes())}:${two(d.getSeconds())}`;
const sleep = ms => new Promise(r => setTimeout(r, ms));

function send(o) {
  const s = JSON.stringify(o);
  for (const c of wss.clients) if (c.readyState === 1) c.send(s);
  const short = o.img ? { ...o, img: o.img.slice(0, 40) + "…" } : o;
  console.log("→ เว็บ", JSON.stringify(short));
}
const log = msg => send({ type: "log", msg });

// ภาพจำลองจากกล้อง (SVG) ให้เห็นว่าเป็นภาพตอนจ่ายหรือตอนหยิบ
function photo(f, s, phase) {
  const label = { dispensed: "ยาลงถาดแล้ว", taken: "หยิบยาแล้ว", manual: "ถ่ายจากเว็บ" }[phase] || phase;
  const bg = { dispensed: "#ffc928", taken: "#16875a", manual: "#1546b8" }[phase] || "#555";
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480"><rect width="640" height="480" fill="${bg}"/>` +
    `<text x="320" y="200" font-size="48" text-anchor="middle" fill="#fff" font-family="sans-serif">ชั้น ${f} · ช่อง ${s}</text>` +
    `<text x="320" y="280" font-size="40" text-anchor="middle" fill="#fff" font-family="sans-serif">${label}</text>` +
    `<text x="320" y="350" font-size="28" text-anchor="middle" fill="#fff" font-family="monospace">${hms(now())} (จำลอง)</text>` +
    (phase === "dispensed" ? `<circle cx="320" cy="420" r="26" fill="#fff"/>` : "") + `</svg>`;
  send({ type: "photo", floor: f, slot: s, phase, at: Date.now(),
         img: "data:image/svg+xml;base64," + Buffer.from(svg).toString("base64") });
}

async function moveTo(f, s) {
  cur = { f, s };
  send({ floor: f, slot: s, state: "moving" });
  await sleep(1200);
  send({ floor: f, slot: s, state: "ready" });
}
async function pushPill() {
  send({ floor: cur.f, slot: cur.s, state: "dispensing" });
  await sleep(1200);
  send({ floor: cur.f, slot: cur.s, state: "ready" });
}

function ring() {
  dose.rings++;
  dose.lastRing = Date.now();
  console.log(`♪ เล่นเพลงเตือน ครั้งที่ ${dose.rings} (ชั้น ${dose.f} ช่อง ${dose.s})`);
}

async function startDose(f, s, scheduled) {
  if (dose) { clearTimeout(dose.timer); log(`ปิดมื้อชั้น ${dose.f} ช่อง ${dose.s} เพราะถึงมื้อใหม่`); }
  await moveTo(f, s);
  await pushPill();
  if (scheduled) floors[f - 1].slots[s - 1].done = "dispensed";
  pill = true;
  dose = { f, s, scheduled, start: Date.now(), lastRing: 0, rings: 0 };
  photo(f, s, "dispensed");
  log(`ยาลงถาดแล้ว ชั้น ${f} ช่อง ${s}`);
  if (scheduled) ring();
  if (TAKE_AFTER > 0) dose.timer = setTimeout(() => { pill = false; }, TAKE_AFTER * 1000);
}

function tick() {
  const n = now();
  // ติดตามมื้อปัจจุบัน
  if (dose) {
    if (!pill) {
      send({ type: "taken", floor: dose.f, slot: dose.s, at: `${isoDate(n)}T${hms(n)}` });
      photo(dose.f, dose.s, "taken");
      if (dose.scheduled) floors[dose.f - 1].slots[dose.s - 1].done = "taken";
      dose = null;
    } else if (dose.scheduled && dose.rings < MAX_RINGS && Date.now() - dose.lastRing >= REMIND * 1000) {
      log(`ยังไม่ได้หยิบยา เตือนครั้งที่ ${dose.rings + 1}`);
      ring();
    } else if (Date.now() - dose.start >= MISSED_MS) {
      log(`ไม่ได้รับยา ชั้น ${dose.f} ช่อง ${dose.s}`);
      dose = null;
    }
  }
  // ถึงเวลาจ่าย?
  const today = isoDate(n), nowMin = n.getHours() * 60 + n.getMinutes();
  floors.forEach((fl, fi) => {
    if (!fl || fl.date !== today) return;
    fl.slots.forEach((sl, si) => {
      if (!sl.on || sl.done) return;
      const due = sl.h * 60 + sl.m;
      if (nowMin >= due && nowMin <= due + 2) {
        sl.done = "busy";
        busy = busy.then(() => startDose(fi + 1, si + 1, true));
      } else if (nowMin > due + 2) {
        sl.done = "missed";
      }
    });
  });
}

function applySchedule(data) {
  const old = floors;
  floors = [];
  let days = 0, count = 0;
  for (const fo of data?.floors || []) {
    if (fo.floor < 1 || fo.floor > 7) continue;
    const prev = old.find(x => x && x.date === fo.date);
    floors[fo.floor - 1] = {
      date: fo.date,
      slots: Array.from({ length: 6 }, (_, i) => {
        const so = (fo.slots || []).find(x => x.slot === i + 1) || {};
        const [h, m] = (so.time || "0:0").split(":").map(Number);
        const on = !!so.on && !!so.time;
        if (on) count++;
        const p = prev?.slots[i];
        const done = p && p.on === on && p.h === h && p.m === m ? p.done : null;
        return { on, h, m, done };
      }),
    };
    days++;
  }
  log(`รับตารางเวลา ${days} วัน ${count} มื้อแล้ว`);
}

function handle(o) {
  const f = o.floor | 0, s = o.slot | 0, ok = f >= 1 && f <= 7 && s >= 1 && s <= 6;
  switch (o.cmd) {
    case "time": {
      const t = new Date(o.y, o.mo - 1, o.d, o.h, o.mi, o.s);
      clockOffset = t - Date.now();
      log(`ตั้งเวลาเครื่องตามเว็บแล้ว ${hms(now())}`);
      break;
    }
    case "schedule": applySchedule(o.data); break;
    case "goto": if (ok) busy = busy.then(() => moveTo(f, s)); break;
    case "dispense": if (ok) busy = busy.then(() => startDose(f, s, false)); break;
    case "capture": if (ok) photo(f, s, "manual"); break;
    case "stop": console.log("■ หยุดเพลง"); break;
    default: log(`ไม่รู้จักคำสั่ง ${o.cmd}`);
  }
}

wss.on("connection", (ws, req) => {
  console.log("เว็บเชื่อมต่อจาก", req.socket.remoteAddress);
  ws.send(JSON.stringify({ type: "log", msg: "กล้อง: เชื่อมต่อเครื่องจ่ายยาแล้ว (จำลอง)" }));
  ws.on("message", buf => {
    let o;
    try { o = JSON.parse(buf.toString()); } catch { return; }
    console.log("← เว็บ", JSON.stringify(o).slice(0, 200));
    handle(o);
  });
});

setInterval(() => send({ type: "status", rtc: hms(now()), floor: cur.f, slot: cur.s, pill }), 2000);
setInterval(tick, 1000);

if (process.stdin.isTTY) {
  process.stdin.setRawMode(true);
  process.stdin.on("data", k => {
    k = k.toString();
    if (k === "t") { pill = false; console.log("(หยิบยาออกจากถาด)"); }
    if (k === "p") { pill = true; console.log("(วางยาในถาด)"); }
    if (k === "q" || k === "\u0003") process.exit(0);
  });
}
console.log(`เครื่องจ่ายยาจำลองพร้อมที่ ws://localhost:${PORT}  (t = หยิบยา, p = วางยา, q = ออก)`);
