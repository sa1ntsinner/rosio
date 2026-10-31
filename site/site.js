// The hero mark lines its third layer up on press; the header takes the ink of the field under it.
const mark = document.querySelector(".inline-mark");
const state = document.querySelector(".state");
const said = {
  false: "Observed is out of line: what runs is not what the launch file says.",
  true: "All four layers agree. rosio has nothing to report.",
};
mark.addEventListener("click", () => {
  const lined = mark.getAttribute("aria-pressed") !== "true";
  mark.setAttribute("aria-pressed", String(lined));
  state.textContent = said[lined];
  mark.closest(".hero").classList.toggle("lined", lined);
});

const bar = document.querySelector(".bar");
const fields = [...document.querySelectorAll("[data-ink]:not(.bar)")];
function inkUnderBar() {
  const y = bar.offsetHeight / 2;
  const under = fields.find((field) => {
    const box = field.getBoundingClientRect();
    return box.top <= y && box.bottom > y;
  });
  if (under) bar.dataset.ink = under.dataset.ink;
}
addEventListener("scroll", inkUnderBar, { passive: true });
inkUnderBar();

const tabs = [...document.querySelectorAll(".names button")];
function show(name) {
  for (const tab of tabs) {
    const on = tab.dataset.sheet === name;
    tab.setAttribute("aria-selected", String(on));
    tab.tabIndex = on ? 0 : -1;
    document.getElementById(`sheet-${tab.dataset.sheet}`).classList.toggle("is-on", on);
  }
}
for (const [i, tab] of tabs.entries()) {
  tab.addEventListener("click", () => show(tab.dataset.sheet));
  tab.addEventListener("keydown", (event) => {
    const step = { ArrowDown: 1, ArrowRight: 1, ArrowUp: -1, ArrowLeft: -1 }[event.key];
    if (!step) return;
    event.preventDefault();
    const next = tabs[(i + step + tabs.length) % tabs.length];
    next.focus();
    show(next.dataset.sheet);
  });
}
show("drift");

const parts = document.querySelectorAll("section h2, .rules li, .shots figure, .how-cols > div, .promises li, .engraving, .releases-shot, .agents-ui, .honest");
if ("IntersectionObserver" in window && !matchMedia("(prefers-reduced-motion: reduce)").matches) {
  parts.forEach((part) => part.classList.add("reveal"));
  const seen = new IntersectionObserver((entries) => {
    for (const entry of entries) {
      if (!entry.isIntersecting) continue;
      const siblings = [...entry.target.parentElement.children].filter((c) => c.classList.contains("reveal"));
      entry.target.style.transitionDelay = `${Math.min(siblings.indexOf(entry.target), 5) * 70}ms`;
      entry.target.classList.add("in");
      seen.unobserve(entry.target);
    }
  }, { rootMargin: "0px 0px -8% 0px" });
  parts.forEach((part) => seen.observe(part));
}
