const root = document.documentElement;
const themeButton = document.querySelector(".theme");

function paintHex() {
  const mode = root.dataset.theme === "light" ? "light" : "dark";
  for (const el of document.querySelectorAll(".hex")) el.textContent = el.dataset[mode];
}

themeButton.addEventListener("click", () => {
  const light = root.dataset.theme !== "light";
  root.dataset.theme = light ? "light" : "dark";
  themeButton.textContent = light ? "Dark" : "Light";
  themeButton.setAttribute("aria-pressed", String(light));
  paintHex();
});
if (location.hash === "#light") themeButton.click();
paintHex();

const says = {
  declared: "The launch file remaps <code>cmd_vel_in</code> on <code>base_driver</code>, and the driver has no endpoint of that name, so the remap does nothing. Edit <code>launch/bringup.launch.py:58</code>.",
  coded: "<code>depth_monitor</code> reads <code>scan_rate</code> with <code>get_parameter</code> and no code declares it, so the node stops on its first read. Edit <code>src/depth_monitor.cpp:27</code>.",
  observed: "The running robot publishes <code>/scan</code> best effort while the source asks for reliable, so <code>depth_monitor</code> never receives a message. Edit <code>src/depth_monitor.cpp:41</code>.",
  recorded: "The bag holds <code>/odom</code> at 11.8 Hz, and the timer that drives it is set to 30 Hz in <code>config/sim.yaml:12</code>. The publisher falls behind its own timer.",
};
const say = document.getElementById("say");
for (const button of document.querySelectorAll(".layer")) {
  button.addEventListener("click", () => {
    for (const other of document.querySelectorAll(".layer")) other.classList.toggle("drift", other === button);
    say.innerHTML = says[button.dataset.layer];
  });
}

if (!matchMedia("(prefers-reduced-motion: reduce)").matches && "IntersectionObserver" in window) {
  const parts = document.querySelectorAll(".sec h2, .sec h3, .sec p, .sec figure, .mark-big, .specimen, .lay, .ground, .steps li, .sizes li, .code-sample, .hero-lead, .plate-say");
  for (const [i, el] of parts.entries()) { el.classList.add("rise"); el.style.transitionDelay = `${(i % 4) * 75}ms`; }
  const seen = new IntersectionObserver((entries) => {
    for (const entry of entries) if (entry.isIntersecting) { entry.target.classList.add("in"); seen.unobserve(entry.target); }
  }, { rootMargin: "0px 0px -8% 0px" });
  for (const el of parts) seen.observe(el);
}
