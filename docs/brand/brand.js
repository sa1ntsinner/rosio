// The hero mark lines its third layer up on press; the header takes the ink of the field under it.
const mark = document.querySelector(".inline-mark");
const state = document.querySelector(".state");
const said = {
  false: "The third layer is out of line: what runs is not what the launch file says.",
  true: "All four layers agree. rosio has nothing to report.",
};
mark?.addEventListener("click", () => {
  const lined = mark.getAttribute("aria-pressed") !== "true";
  mark.setAttribute("aria-pressed", String(lined));
  state.textContent = said[lined];
  mark.closest(".hero").classList.toggle("lined", lined);
});

const bar = document.querySelector(".bar");
const fields = [...document.querySelectorAll("[data-ink]:not(.bar)")];
const inkUnderBar = () => {
  const y = bar.offsetHeight / 2;
  const under = fields.find((f) => {
    const r = f.getBoundingClientRect();
    return r.top <= y && r.bottom > y;
  });
  if (under) bar.dataset.ink = under.dataset.ink;
};
addEventListener("scroll", inkUnderBar, { passive: true });
inkUnderBar();

const parts = document.querySelectorAll("section h2, section .say p, section .facts li, section figure, section .cols > *, .fields .field, .stack > div, .severity > div, .specimen, .rules > div, .wide-lock, .note");
if ("IntersectionObserver" in window && !matchMedia("(prefers-reduced-motion: reduce)").matches) {
  parts.forEach((el) => el.classList.add("reveal"));
  const seen = new IntersectionObserver((entries) => {
    for (const entry of entries) {
      if (!entry.isIntersecting) continue;
      const siblings = [...entry.target.parentElement.children].filter((c) => c.classList.contains("reveal"));
      entry.target.style.transitionDelay = `${Math.min(siblings.indexOf(entry.target), 5) * 70}ms`;
      entry.target.classList.add("in");
      seen.unobserve(entry.target);
    }
  }, { rootMargin: "0px 0px -8% 0px" });
  parts.forEach((el) => seen.observe(el));
}
