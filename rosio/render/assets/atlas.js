"use strict";
// atlas.html. Both data blocks live in this file as gzip in base64 and nothing is fetched.
// They are unpacked before the page starts, the index is parsed then, the detail on the
// first click that needs it.
function unpack(id) {
  var block = document.getElementById(id);
  if (!block) return Promise.resolve("");
  var bytes = Uint8Array.from(atob(block.textContent), function (c) { return c.charCodeAt(0); });
  var stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream("gzip"));
  return new Response(stream).text();
}

// The detail block sits after this script in the page, so nothing is read before the
// document is whole: read at once, the block is not there yet and every card is empty.
function boot() {
  Promise.all([unpack("rosio-index"), unpack("rosio-detail")]).then(function (texts) {
    start(texts[0], texts[1]);
  });
}
if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", boot);
else boot();

function start(indexText, detailText) {
  var IX = JSON.parse(indexText);
  var E = IX.entities;
  var N = E.id.length;
  var DETAIL = null;

  function detail() {
    if (DETAIL === null) {
      DETAIL = detailText ? JSON.parse(detailText) : { entities: {}, unresolved: {} };
      detailText = "";
    }
    return DETAIL;
  }
  function facts(index) {
    return detail().entities[E.id[index]] || {};
  }

  var COLOUR = {
    agree: "#b39780",
    differ: "#ec5a55",
    declared: "#4a92ea",
    coded: "#7a4eb8",
    observed: "#2b8044",
    recorded: "#d8609f",
    none: "#3a3a3a"
  };
  // a parameter state in the words of the person who set it; an effective one needs none
  var STATE = {
    inert_by_design: "its plugin is not selected",
    undeclared: "set and never declared",
    declared_unread: "declared and never read",
    owner_name_mismatch: "no node of this name runs",
    unresolved_owner: "its node could not be followed",
    other: "no value wins"
  };
  // what a person calls each layer: the thing it is read from
  var LAYER = { declared: "config", coded: "code", observed: "live", recorded: "bag" };
  function lname(layer) { return LAYER[layer] || layer; }
  // how a value got where it is, in words
  var MECHANISM = {
    launch_rewritten_yaml: "yaml rewritten by the launch file",
    launch_inline_dict: "inline parameters of the launch file",
    launch_set_parameter: "SetParameter for the whole launch group",
    launch_params_file: "parameters file passed by the launch file",
    launch_yaml: "parameters file passed by the launch file",
    yaml: "parameters file",
    code_default: "default in the source",
    code_set: "set by the node itself",
    ros_args: "command line argument",
    python_override: "rewritten by a launcher script",
    param_set_runtime: "ros2 param set after start"
  };
  var WHY = {
    value_changed: "a later write set a different value",
    restated_identical: "a later write set the same value again",
    type_changed: "a later write changed the type",
    not_read_in_profile: "the file it sits in is not read in this profile",
    inert_plugin_block: "the plugin instance this block configures is not selected",
    other: "a later write took precedence"
  };

  // ---------------------------------------------------------------- helpers
  function el(id) { return document.getElementById(id); }
  function esc(text) {
    return String(text).replace(/[&<>"]/g, function (ch) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch];
    });
  }
  function bit(mask, index) { return (mask & (1 << index)) !== 0; }
  function fpath(id) {
    if (IX.paths[id]) return IX.paths[id];
    var cut = String(id).indexOf(":");
    if (cut < 0) return String(id);
    var body = id.slice(cut + 1).split("|").join("/");
    return body.indexOf("@repo/") === 0 ? body.slice(6) : body;
  }
  function cut(text, limit) {
    return text.length > limit ? text.slice(0, limit - 1) + "\u2026" : text;
  }
  // `applied_by` is a sym id. Joining its qualified name onto the path with a slash
  // reads as a directory that does not exist, so the name is kept beside the file.
  function symText(id) {
    var text = String(id);
    var parts = text.slice(text.indexOf(":") + 1).split("|");
    if (text.indexOf("sym:") !== 0 || parts.length < 3) return fpath(text);
    var name = parts.pop();
    return fpath("file:" + parts.join("|")) + " " + name;
  }
  function shortName(id) {
    var body = String(id).split(":").slice(1).join(":");
    var parts = body.split("|");
    return parts[parts.length - 1] || body;
  }
  function profileName(id) {
    for (var i = 0; i < IX.profiles.length; i++) {
      if (IX.profiles[i].id === id) return shortName(id);
    }
    return shortName(id);
  }
  function valueText(value) {
    if (value === null || value === undefined) return "unknown";
    if (typeof value === "object") return JSON.stringify(value);
    if (typeof value === "string") return value;
    return String(value);
  }
  function shownValue(value, type) {
    // The atlas keeps an int an int and a float a float. JSON gives back one number
    // type, so the float mark is put back rather than printing 1 for 1.0. The C++ side
    // spells the type "double" and the python side "float", and both are one here.
    var text = valueText(value);
    if ((type === "float" || type === "double") && typeof value === "number" &&
        text.indexOf(".") < 0 && text.indexOf("e") < 0) text += ".0";
    return text;
  }
  function plural(count, word, many) {
    return count + " " + (count === 1 ? word : (many || word + "s"));
  }
  function profileSlot() {
    for (var p = 0; p < IX.profiles.length; p++) {
      if (IX.profiles[p].id === state.profile) return p;
    }
    return -1;
  }

  function source(row) {
    if (!row) return "";
    var line = row.line_hint === undefined ? row.line : row.line_hint;
    var at = fpath(row.file || "") + (line !== null && line !== undefined ? ":" + line : "");
    var out = '<span class="src" data-copy="' + esc(at) + '" title="Copy ' + esc(at) + '"><b>' + esc(fpath(row.file || "")) + "</b>";
    if (line !== null && line !== undefined) out += '<span class="ln">:' + line + "</span>";
    if (row.locator) out += " " + esc(row.locator);
    return out + "</span>";
  }

  // ---------------------------------------------------------------- state
  var state = {
    view: "",
    layers: {},
    kinds: {},
    profile: "*",
    finding: "any",
    query: "",
    selected: -1,
    finding_index: -1,
    param: -1,
    chip: "all",
    sev: "all",
    rule: ""
  };
  var vis = new Uint8Array(N);
  var shownCount = 0;
  var hit = new Uint8Array(N);

  IX.layer_names.forEach(function (name) { state.layers[name] = true; });
  // The graph opens on what runs and what it says. Files, symbols, parameters and the
  // bookkeeping of the extraction are one tick away in the kind list.
  var GRAPH_KINDS = ["node", "topic", "service", "action", "frame", "bag", "bagtopic"];
  IX.kinds.forEach(function (name) { state.kinds[name] = GRAPH_KINDS.indexOf(name) >= 0; });

  var findingOf = new Int8Array(N);
  for (var i = 0; i < N; i++) findingOf[i] = E.finding[i];

  function passes(index, anyKind) {
    if (!anyKind && !state.kinds[IX.kinds[E.kind[index]]]) return false;
    var mask = E.layers[index];
    var any = mask === 0;
    for (var l = 0; l < IX.layer_names.length && !any; l++) {
      if (bit(mask, l) && state.layers[IX.layer_names[l]]) any = true;
    }
    if (!any) return false;
    // An empty profile mask is a fact stated under `*`, which holds in every profile.
    // It is what the edge filter in draw() already does and what a profile slice of the
    // atlas keeps, so a file, a type or a plugin parameter must not vanish here.
    if (state.profile !== "*") {
      var slot = profileSlot();
      var profiles = E.profiles[index];
      if (slot >= 0 && profiles !== 0 && !bit(profiles, slot) && E.id[index] !== state.profile) {
        return false;
      }
    }
    if (state.finding === "yes" && E.finding[index] === 0) return false;
    if (state.finding === "no" && E.finding[index] !== 0) return false;
    return true;
  }

  function recompute() {
    var query = state.query.toLowerCase();
    var shown = 0;
    for (var i = 0; i < N; i++) {
      var ok = passes(i) ? 1 : 0;
      vis[i] = ok;
      hit[i] = ok && query && (E.label[i].toLowerCase().indexOf(query) >= 0 ||
        E.id[i].toLowerCase().indexOf(query) >= 0) ? 1 : 0;
      shown += ok;
    }
    shownCount = shown;
    el("shown").textContent = query ? plural(countHits(), "match", "matches") : "";
    draw();
  }
  function countHits() {
    var total = 0;
    for (var i = 0; i < N; i++) total += hit[i];
    return total;
  }

  // ---------------------------------------------------------------- colours
  function colourOf(index) {
    var mask = E.layers[index];
    if (mask === 0) return COLOUR.none;
    var count = 0;
    var only = -1;
    for (var l = 0; l < IX.layer_names.length; l++) {
      if (bit(mask, l)) { count++; only = l; }
    }
    var differs = IX.agreements[E.agreement[index]] === "differ" || (E.flags[index] & 1) !== 0;
    if (differs) return COLOUR.differ;
    if (count > 1) return COLOUR.agree;
    return COLOUR[IX.layer_names[only]] || COLOUR.none;
  }
  function isHub(index) {
    var kind = IX.kinds[E.kind[index]];
    return kind === "node" || kind === "pkg" || kind === "profile" || kind === "bag";
  }

  // ---------------------------------------------------------------- graph
  var canvas = el("graph");
  var ctx = canvas.getContext("2d");
  var view = { scale: 1, x: 0, y: 0 };
  var dpr = 1;

  function resize() {
    dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(canvas.clientWidth * dpr));
    canvas.height = Math.max(1, Math.round(canvas.clientHeight * dpr));
    draw();
  }

  function fit() {
    var minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
    for (var i = 0; i < N; i++) {
      if (!vis[i]) continue;
      if (E.x[i] < minx) minx = E.x[i];
      if (E.y[i] < miny) miny = E.y[i];
      if (E.x[i] > maxx) maxx = E.x[i];
      if (E.y[i] > maxy) maxy = E.y[i];
    }
    if (!isFinite(minx)) { minx = miny = 0; maxx = maxy = 1; }
    var w = canvas.clientWidth || 1;
    var h = canvas.clientHeight || 1;
    var scale = Math.min(w / (maxx - minx + 80), h / (maxy - miny + 80));
    view.scale = scale;
    view.x = w / 2 - ((minx + maxx) / 2) * scale;
    view.y = h / 2 - ((miny + maxy) / 2) * scale;
    draw();
  }

  function draw() {
    if (state.view !== "graph") return;
    var w = canvas.clientWidth, h = canvas.clientHeight;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#111111";
    ctx.fillRect(0, 0, w, h);
    var s = view.scale, ox = view.x, oy = view.y;

    var edges = IX.edges;
    var pslot = -1;
    for (var p = 0; p < IX.profiles.length; p++) {
      if (IX.profiles[p].id === state.profile) pslot = p;
    }
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(150,170,175,0.10)";
    ctx.beginPath();
    var focus = state.selected;
    var linked = [];
    for (var e = 0; e < edges.length; e += 4) {
      var a = edges[e], b = edges[e + 1];
      if (!vis[a] || !vis[b]) continue;
      if (pslot >= 0 && edges[e + 3] !== 0 && !bit(edges[e + 3], pslot)) continue;
      if (focus >= 0 && (a === focus || b === focus)) { linked.push(e); continue; }
      ctx.moveTo(E.x[a] * s + ox, E.y[a] * s + oy);
      ctx.lineTo(E.x[b] * s + ox, E.y[b] * s + oy);
    }
    ctx.stroke();
    if (linked.length) {
      ctx.strokeStyle = "rgba(121,189,3,0.65)";
      ctx.beginPath();
      for (var k = 0; k < linked.length; k++) {
        var ea = edges[linked[k]], eb = edges[linked[k] + 1];
        ctx.moveTo(E.x[ea] * s + ox, E.y[ea] * s + oy);
        ctx.lineTo(E.x[eb] * s + ox, E.y[eb] * s + oy);
      }
      ctx.stroke();
    }

    var groups = {};
    for (var i = 0; i < N; i++) {
      if (!vis[i]) continue;
      var colour = colourOf(i);
      (groups[colour] || (groups[colour] = [])).push(i);
    }
    var dim = state.query.length > 0;
    Object.keys(groups).sort().forEach(function (colour) {
      ctx.fillStyle = colour;
      var list = groups[colour];
      for (var j = 0; j < list.length; j++) {
        var index = list[j];
        if (dim && !hit[index]) continue;
        var size = isHub(index) ? 7 : 4;
        ctx.fillRect(E.x[index] * s + ox - size / 2, E.y[index] * s + oy - size / 2, size, size);
      }
    });
    if (dim) {
      ctx.fillStyle = "rgba(150,160,164,0.18)";
      for (var d = 0; d < N; d++) {
        if (!vis[d] || hit[d]) continue;
        ctx.fillRect(E.x[d] * s + ox - 1.5, E.y[d] * s + oy - 1.5, 3, 3);
      }
    }

    for (var f = 0; f < N; f++) {
      if (!vis[f] || findingOf[f] === 0) continue;
      ctx.strokeStyle = findingOf[f] === 3 ? "#ec5a55" : (findingOf[f] === 2 ? "#e8a33d" : "#6e6e6e");
      ctx.lineWidth = findingOf[f] === 3 ? 1.8 : 1.1;
      ctx.beginPath();
      ctx.arc(E.x[f] * s + ox, E.y[f] * s + oy, 6, 0, 6.2832);
      ctx.stroke();
    }

    if (highlight.length) {
      ctx.strokeStyle = "#ec5a55";
      ctx.lineWidth = 2;
      for (var g = 0; g < highlight.length; g++) {
        var hgt = highlight[g];
        ctx.beginPath();
        ctx.arc(E.x[hgt] * s + ox, E.y[hgt] * s + oy, 11, 0, 6.2832);
        ctx.stroke();
      }
    }
    if (state.selected >= 0) {
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(E.x[state.selected] * s + ox, E.y[state.selected] * s + oy, 9, 0, 6.2832);
      ctx.stroke();
    }

    // A cluster is named once it is wide enough on screen to read, so zooming out
    // leaves the shape and zooming in fills in the names.
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillStyle = "#d9d9d9";
    // hubs are named first, and a name that would sit on one already drawn is left out
    var placed = [];
    var order = [];
    for (var t = 0; t < N; t++) {
      if (!vis[t]) continue;
      var named = E.radius[t] * s > 22 || shownCount <= 60;
      if (!named && t !== state.selected && !hit[t]) continue;
      if (dim && !hit[t] && t !== state.selected) continue;
      order.push(t);
    }
    order.sort(function (a, b) {
      return (b === state.selected) - (a === state.selected) || isHub(b) - isHub(a) || a - b;
    });
    order.forEach(function (t) {
      var text = cut(E.label[t], 34);
      var x = E.x[t] * s + ox + 8, y = E.y[t] * s + oy + 3;
      var wide = ctx.measureText(text).width;
      for (var q = 0; q < placed.length; q++) {
        var box = placed[q];
        if (x < box[0] + box[2] + 4 && x + wide + 4 > box[0] && Math.abs(y - box[1]) < 13) return;
      }
      placed.push([x, y, wide]);
      ctx.fillText(text, x, y);
    });
  }

  var highlight = [];
  var fitted = false;

  function nearest(px, py) {
    var best = -1, bestDistance = 18 * 18;
    for (var i = 0; i < N; i++) {
      if (!vis[i]) continue;
      var dx = E.x[i] * view.scale + view.x - px;
      var dy = E.y[i] * view.scale + view.y - py;
      var distance = dx * dx + dy * dy;
      if (distance < bestDistance) { bestDistance = distance; best = i; }
    }
    return best;
  }

  var dragging = false, moved = false, lastX = 0, lastY = 0;
  canvas.addEventListener("pointerdown", function (event) {
    dragging = true; moved = false; lastX = event.clientX; lastY = event.clientY;
    canvas.classList.add("drag");
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", function (event) {
    var rect = canvas.getBoundingClientRect();
    if (dragging) {
      var dx = event.clientX - lastX, dy = event.clientY - lastY;
      if (Math.abs(dx) + Math.abs(dy) > 2) moved = true;
      view.x += dx; view.y += dy;
      lastX = event.clientX; lastY = event.clientY;
      draw();
      return;
    }
    var index = nearest(event.clientX - rect.left, event.clientY - rect.top);
    var tip = el("tip");
    if (index < 0) { tip.hidden = true; return; }
    tip.hidden = false;
    tip.textContent = IX.kinds[E.kind[index]] + "  " + E.label[index];
    tip.style.left = (event.clientX - rect.left + 14) + "px";
    tip.style.top = (event.clientY - rect.top + 12) + "px";
  });
  canvas.addEventListener("pointerup", function (event) {
    dragging = false;
    canvas.classList.remove("drag");
    if (moved) return;
    var rect = canvas.getBoundingClientRect();
    var index = nearest(event.clientX - rect.left, event.clientY - rect.top);
    if (index >= 0) select(index);
  });
  canvas.addEventListener("wheel", function (event) {
    event.preventDefault();
    var rect = canvas.getBoundingClientRect();
    var px = event.clientX - rect.left, py = event.clientY - rect.top;
    var factor = Math.exp(-event.deltaY * 0.0015);
    var next = Math.min(24, Math.max(0.02, view.scale * factor));
    view.x = px - (px - view.x) * (next / view.scale);
    view.y = py - (py - view.y) * (next / view.scale);
    view.scale = next;
    draw();
  }, { passive: false });

  function centreOn(index) {
    view.scale = Math.max(view.scale, 1.4);
    view.x = canvas.clientWidth / 2 - E.x[index] * view.scale;
    view.y = canvas.clientHeight / 2 - E.y[index] * view.scale;
    draw();
  }

  // ---------------------------------------------------------------- panel
  function select(index) {
    var kind = IX.kinds[E.kind[index]];
    if (!state.kinds[kind]) {
      // a thing picked from a list is shown in the graph even when its kind was off
      state.kinds[kind] = true;
      buildFilters();
      recompute();
    }
    state.selected = index;
    renderPanel(index);
    draw();
  }

  function chips(index) {
    var out = [];
    var mask = E.layers[index];
    for (var l = 0; l < IX.layer_names.length; l++) {
      if (bit(mask, l)) out.push('<span class="tag layer">' + lname(IX.layer_names[l]) + "</span>");
    }
    var agreement = IX.agreements[E.agreement[index]];
    if (agreement === "agree") out.push('<span class="tag agree">layers agree</span>');
    if (agreement === "differ") out.push('<span class="tag differ">layers differ</span>');
    for (var p = 0; p < IX.profiles.length; p++) {
      if (bit(E.profiles[index], p)) {
        out.push('<span class="tag">' + esc(shortName(IX.profiles[p].id)) + "</span>");
      }
    }
    var stated = E.state[index] >= 0 ? STATE[IX.states[E.state[index]]] : "";
    if (stated) out.push('<span class="tag differ">' + stated + "</span>");
    return '<div class="tags">' + out.join("") + "</div>";
  }

  function kvTable(values) {
    var keys = Object.keys(values || {}).sort();
    if (!keys.length) return '<p class="empty">no values</p>';
    var rows = keys.map(function (key) {
      return '<tr><td class="k">' + esc(key) + '</td><td class="v">' + esc(valueText(values[key])) + "</td></tr>";
    });
    return '<table class="kv">' + rows.join("") + "</table>";
  }

  function renderPanel(index) {
    el("panelbody").innerHTML = panelHtml(index, true);
  }

  function panelHtml(index, head) {
    var data = facts(index);
    var out = [];
    if (head) {
      out.push('<h1 class="title">' + esc(E.label[index]) + "</h1>");
      out.push('<p class="eyebrow">' + esc(IX.kinds[E.kind[index]]) + "  " + esc(E.id[index]) + "</p>");
      out.push(chips(index));
    }

    var conflicts = data.conflicts || [];
    if (conflicts.length) {
      out.push("<h3>conflicts</h3><p class=\"tags\">" + conflicts.map(function (name) {
        return '<span class="tag differ">' + esc(name) + "</span>";
      }).join("") + "</p>");
    }

    if (data.candidates && data.candidates.length) {
      out.push('<h3>parameter chain</h3><p><span class="link" data-chain="' + index +
        '">open the chain, ' + data.candidates.length +
        (data.candidates.length === 1 ? " candidate" : " candidates") + "</span></p>");
    }

    var effective = data.effective || {};
    var profiles = Object.keys(effective).sort();
    if (profiles.length) {
      out.push("<h3>effective</h3>");
      profiles.forEach(function (profile) {
        out.push('<div class="card"><div class="meta">' + esc(profileName(profile)) + "</div>");
        var keys = Object.keys(effective[profile]).sort();
        keys.forEach(function (key) {
          var entry = effective[profile][key];
          out.push('<div class="value">' + esc(key) + " = " +
            esc(shownValue(entry.value, entry.value_type)) + "</div>");
          out.push('<div class="meta">' + lname(entry.layer) + (entry.value_type ? ", " + entry.value_type : "") +
            (entry.source ? " &middot; " + source(entry.source) : "") + "</div>");
        });
        out.push("</div>");
      });
    }

    var list = (data.facts || []).slice().sort(function (a, b) {
      return (a.layer + a.profile).localeCompare(b.layer + b.profile);
    });
    if (list.length) {
      out.push("<h3>facts</h3>");
      list.forEach(function (fact) {
        out.push('<div class="card"><div class="meta">' + esc(lname(fact.layer)) + " &middot; " +
          esc(profileName(fact.profile)) + " &middot; " + esc(fact.confidence) + "</div>");
        out.push(kvTable(fact.values));
        (fact.evidence || []).forEach(function (row) {
          out.push("<div>" + source(row) + "</div>");
        });
        out.push("</div>");
      });
    }

    var blind = detail().unresolved[E.id[index]] || [];
    if (blind.length) {
      out.push("<h3>unresolved</h3>");
      blind.forEach(function (entry) {
        out.push('<div class="card"><div class="value">' + esc(entry.want) + ": " + esc(entry.expr) + "</div>" +
          '<div class="meta">' + esc(entry.reason) + "</div>" +
          (entry.evidence || []).map(function (row) { return "<div>" + source(row) + "</div>"; }).join("") +
          "</div>");
      });
    }

    var about = [];
    IX.findings.forEach(function (finding, position) {
      if (finding.subjects.indexOf(index) >= 0) about.push(position);
    });
    if (about.length) {
      out.push("<h3>findings</h3>");
      about.forEach(function (position) {
        var finding = IX.findings[position];
        out.push('<div class="frow" data-finding="' + position + '"><div class="head">' +
          '<span class="sev ' + finding.severity + '">' + finding.severity + "</span>" +
          '<span class="rule">' + esc(finding.rule) + "</span></div>" +
          '<div class="say">' + esc(finding.say || finding.title) + "</div></div>");
      });
    }

    out.push("<h3>edges</h3>");
    var neighbours = [];
    var edges = IX.edges;
    for (var e = 0; e < edges.length; e += 4) {
      if (edges[e] === index) neighbours.push(["out", IX.edge_kinds[edges[e + 2]], edges[e + 1]]);
      else if (edges[e + 1] === index) neighbours.push(["in", IX.edge_kinds[edges[e + 2]], edges[e]]);
      if (neighbours.length > 400) break;
    }
    if (!neighbours.length) out.push('<p class="empty">none</p>');
    else {
      out.push('<div class="edges">' + neighbours.map(function (row) {
        return "<div><span class=\"k\">" + (row[0] === "out" ? "&rarr; " : "&larr; ") + esc(row[1]) +
          '</span><span class="link" data-go="' + row[2] + '">' + esc(E.label[row[2]]) + "</span></div>";
      }).join("") + "</div>");
    }

    return out.join("");
  }

  // ---------------------------------------------------------------- chain
  function renderChain(index) {
    state.param = index;
    var data = facts(index);
    var candidates = (data.candidates || []).slice();
    var out = [];
    var owner = E.id[index].slice(E.id[index].indexOf(":") + 1);
    owner = decodeURIComponent(owner.slice(0, owner.lastIndexOf("|")));
    out.push("<h2>" + esc(E.label[index]) + "</h2>");
    out.push('<p class="eyebrow">' + esc(owner.charAt(0) === "~" ? "an unnamed node" : owner) + "</p>");
    out.push(chips(index));
    if (!candidates.length) {
      // no chain to show, so the column holds what the layers state about it
      el("chain").innerHTML = out.join("") + panelHtml(index, false);
      return;
    }
    var byProfile = {};
    candidates.forEach(function (candidate) {
      (byProfile[candidate.profile] || (byProfile[candidate.profile] = [])).push(candidate);
    });
    Object.keys(byProfile).sort().forEach(function (profile) {
      var rows = byProfile[profile].slice().sort(function (a, b) { return a.rank - b.rank; });
      var winner = rows.filter(function (row) { return row.wins; })[0];
      out.push('<div class="profile' + (winner ? "" : " blind") + '">' + esc(profileName(profile)) +
        (winner ? "" : " &middot; nothing wins here, the profile could not follow this parameter") + "</div>");
      rows.forEach(function (row) {
        var lost = !row.wins;
        var why = lost ? (WHY[row.shadow_kind] || WHY.other) : "";
        if (lost && row.shadowed_by !== undefined && row.shadowed_by !== null) {
          var over = rows.filter(function (item) { return item.rank === row.shadowed_by; })[0];
          if (over) why += ", " + esc(fpath(over.source.file)) +
            (over.source.line_hint ? ":" + over.source.line_hint : "") + " sets " +
            esc(shownValue(over.value, over.value_type));
        }
        out.push('<div class="card ' + (lost ? "lost" : "win") + '">');
        out.push('<div class="meta">' + esc(MECHANISM[row.mechanism] || String(row.mechanism).replace(/_/g, " ")) +
          (row.wins ? ' &middot; <span class="win-mark">wins</span>' : "") + "</div>");
        out.push('<div class="value">' + esc(shownValue(row.value, row.value_type)) +
          ' <span class="meta">' + esc(row.value_type) + "</span></div>");
        out.push("<div>" + source(row.source) + "</div>");
        if (row.applied_by) out.push('<div class="meta">applied by ' + esc(symText(row.applied_by)) + "</div>");
        if (row.guard) out.push('<div class="meta">guarded by ' + esc(row.guard) + "</div>");
        if (lost) out.push('<div class="why">' + why + "</div>");
        out.push("</div>");
      });
    });
    el("chain").innerHTML = out.join("");
  }

  // ---------------------------------------------------------------- lists
  var paramIndexes = [];
  for (var pi = 0; pi < N; pi++) {
    var kindName = IX.kinds[E.kind[pi]];
    if (kindName === "param" || kindName === "pparam") paramIndexes.push(pi);
  }

  function shadowed(index, slot) {
    return slot >= 0 ? bit(E.shadow[index], slot) : (E.flags[index] & 4) !== 0;
  }

  // The value column answers for the selected profile. Two profiles that configure one
  // parameter differently have two answers, so with every profile shown it names neither.
  function valueCell(index, slot) {
    var per = IX.value_by_profile[index];
    if (!per) return E.value[index] ? esc(E.value[index]) : "";
    if (slot < 0) return '<span class="meta">per profile</span>';
    return per[slot] ? esc(per[slot]) : "";
  }

  function renderParams() {
    var query = el("psearch").value.toLowerCase();
    var slot = profileSlot();
    var out = [];
    var count = 0;
    for (var i = 0; i < paramIndexes.length && count < 400; i++) {
      var index = paramIndexes[i];
      if (!passes(index, true)) continue;
      if (query && E.id[index].toLowerCase().indexOf(query) < 0) continue;
      if (state.chip === "shadowed" && !shadowed(index, slot)) continue;
      if (state.chip === "finding" && E.finding[index] === 0) continue;
      if (state.chip === "inert" && IX.states[E.state[index]] === "effective") continue;
      count++;
      var owner = E.id[index].slice(E.id[index].indexOf(":") + 1);
      owner = owner.slice(0, owner.lastIndexOf("|"));
      var cell = valueCell(index, slot);
      out.push('<div class="row' + (shadowed(index, slot) ? " shadow" : "") +
        (state.param === index ? " on" : "") + '" data-param="' + index + '">' +
        '<div class="nm">' + esc(E.label[index]) + "</div>" +
        '<div class="sub"><span>' + esc(owner) + "</span></div>" +
        (cell ? '<div class="val">' + cell + "</div>" : "") + "</div>");
    }
    if (!count) out.push('<div class="more">nothing matches</div>');
    else out.push('<div class="more">' + count + (count === 400 ? "+" : "") + " shown</div>");
    el("prows").innerHTML = out.join("");
  }

  function renderFindings() {
    var out = [];
    if (state.rule) {
      out.push('<p class="empty">rule ' + esc(state.rule) +
        ' only, <span class="link" data-rule="">show every rule</span></p>');
    }
    IX.findings.forEach(function (finding, position) {
      if (state.sev !== "all" && finding.severity !== state.sev) return;
      if (state.rule && finding.rule !== state.rule) return;
      out.push('<div class="frow' + (state.finding_index === position ? " on" : "") +
        '" data-finding="' + position + '"><div class="head">' +
        '<span class="sev ' + finding.severity + '">' + finding.severity + "</span>" +
        '<span class="ttl">' + esc(finding.title) + "</span>" +
        '<span class="rule">' + esc(finding.rule) + "</span></div>" +
        '<div class="body"><div class="say">' + esc(finding.say || finding.detail) + "</div>" +
        (finding.fix ? '<div class="fix">' + esc(finding.fix) + "</div>" : "") +
        (finding.where.length ? '<div class="where">' + finding.where.map(function (row) {
          return source({ file: row.file, line: row.line });
        }).join("") + "</div>" : "") + "</div></div>");
    });
    if (!out.length) out.push('<p class="empty">no findings at this severity</p>');
    el("frows").innerHTML = out.join("");
  }

  function pickFinding(position) {
    state.finding_index = position;
    var finding = IX.findings[position];
    highlight = finding.subjects.slice();
    renderFindings();
    if (highlight.length) {
      var first = highlight[0];
      state.selected = first;
      renderPanel(first);
      var flags = E.flags[first];
      if (flags & 16) {
        el("psearch").value = E.label[first];
        showView("params");
        renderChain(first);
      }
      else { centreOn(first); }
    }
    draw();
  }

  // ---------------------------------------------------------------- views
  function showView(name) {
    state.view = name;
    document.body.setAttribute("data-view", name);
    ["graph", "params", "findings"].forEach(function (item) {
      el("view-" + item).classList.toggle("on", item === name);
    });
    Array.prototype.forEach.call(document.querySelectorAll(".tabs button"), function (button) {
      button.classList.toggle("on", button.getAttribute("data-view") === name);
    });
    if (name === "params") renderParams();
    if (name === "findings") renderFindings();
    if (name === "graph") {
      resize();
      // the canvas has no size until it is first shown
      if (!fitted) { fitted = true; fit(); }
    }
  }

  // ---------------------------------------------------------------- filters
  function buildFilters() {
    var counts = {};
    for (var i = 0; i < N; i++) {
      var kind = IX.kinds[E.kind[i]];
      counts[kind] = (counts[kind] || 0) + 1;
    }
    el("f-kinds").innerHTML = IX.kinds.map(function (kind) {
      return '<label class="check"><input type="checkbox"' + (state.kinds[kind] ? " checked" : "") +
        ' data-kind="' + kind + '">' +
        '<span>' + esc(kind) + '</span><span class="n">' + counts[kind] + "</span></label>";
    }).join("");
    el("f-layers").innerHTML = IX.layer_names.map(function (layer) {
      return '<label class="check"><input type="checkbox" checked data-layer="' + layer + '">' +
        "<span>" + esc(lname(layer)) + "</span></label>";
    }).join("");
    el("f-profiles").innerHTML = ['<label class="check"><input type="radio" name="prof" value="*" checked><span>every profile</span></label>']
      .concat(IX.profiles.map(function (profile) {
        return '<label class="check"><input type="radio" name="prof" value="' + esc(profile.id) + '"><span>' +
          esc(shortName(profile.id)) + "</span></label>";
      })).join("");
    el("f-finding").innerHTML = ["any", "yes", "no"].map(function (mode) {
      return '<label class="check"><input type="radio" name="find" value="' + mode + '"' +
        (mode === "any" ? " checked" : "") + "><span>" +
        (mode === "any" ? "any entity" : (mode === "yes" ? "carries a finding" : "carries none")) +
        "</span></label>";
    }).join("");
  }

  function renderMatches() {
    if (!state.query) { el("matches").innerHTML = ""; return; }
    var out = [];
    for (var i = 0; i < N && out.length < 60; i++) {
      if (hit[i]) {
        out.push('<div data-go="' + i + '"><i>' + IX.kinds[E.kind[i]] + "</i>" + esc(E.label[i]) + "</div>");
      }
    }
    el("matches").innerHTML = out.join("") || '<div><i>none</i></div>';
  }

  // ---------------------------------------------------------------- wiring
  document.addEventListener("click", function (event) {
    var selector = "[data-copy],[data-go],[data-finding],[data-param],[data-chain],[data-view],[data-chip],[data-sev],[data-rule]";
    var target = event.target.closest ? event.target.closest(selector) : null;
    if (!target) return;
    if (target.hasAttribute("data-copy")) {
      if (navigator.clipboard) navigator.clipboard.writeText(target.getAttribute("data-copy"));
      target.classList.add("copied");
      setTimeout(function () { target.classList.remove("copied"); }, 900);
      return;
    }
    if (target.hasAttribute("data-view")) { showView(target.getAttribute("data-view")); return; }
    if (target.hasAttribute("data-go")) {
      var index = parseInt(target.getAttribute("data-go"), 10);
      if (state.view !== "graph") showView("graph");
      select(index); centreOn(index); return;
    }
    if (target.hasAttribute("data-chain")) {
      showView("params");
      renderChain(parseInt(target.getAttribute("data-chain"), 10));
      renderParams();
      return;
    }
    if (target.hasAttribute("data-param")) {
      var pick = parseInt(target.getAttribute("data-param"), 10);
      renderChain(pick);
      state.selected = pick;
      renderPanel(pick);
      renderParams();
      return;
    }
    if (target.hasAttribute("data-rule")) {
      state.rule = target.getAttribute("data-rule");
      showView("findings");
      return;
    }
    if (target.hasAttribute("data-finding")) {
      pickFinding(parseInt(target.getAttribute("data-finding"), 10));
      return;
    }
    if (target.hasAttribute("data-chip")) {
      state.chip = target.getAttribute("data-chip");
      Array.prototype.forEach.call(el("pchips").children, function (button) {
        button.classList.toggle("on", button === target);
      });
      renderParams();
      return;
    }
    if (target.hasAttribute("data-sev")) {
      state.sev = target.getAttribute("data-sev");
      state.rule = "";
      Array.prototype.forEach.call(el("fchips").children, function (button) {
        button.classList.toggle("on", button === target);
      });
      renderFindings();
    }
  });

  el("side").addEventListener("change", function (event) {
    var input = event.target;
    if (input.hasAttribute("data-kind")) state.kinds[input.getAttribute("data-kind")] = input.checked;
    else if (input.hasAttribute("data-layer")) state.layers[input.getAttribute("data-layer")] = input.checked;
    else if (input.name === "prof") state.profile = input.value;
    else if (input.name === "find") state.finding = input.value;
    recompute();
    if (state.view === "params") renderParams();
  });

  el("search").addEventListener("input", function (event) {
    state.query = event.target.value.trim();
    recompute();
    renderMatches();
  });
  el("psearch").addEventListener("input", renderParams);
  el("fit").addEventListener("click", fit);
  window.addEventListener("resize", resize);
  document.addEventListener("keydown", function (event) {
    if (event.key === "Escape") { state.selected = -1; highlight = []; draw(); }
  });

  // ---------------------------------------------------------------- start
  el("counts").textContent = IX.profiles.map(function (profile) { return shortName(profile.id); })
    .concat(IX.layer_names.length ? [IX.layer_names.map(lname).join(", ")] : []).join(" \u00b7 ");
  el("findcount").textContent = IX.findings.length || "";
  buildFilters();
  recompute();
  // what is wrong comes first; a clean atlas opens on the graph
  // and a link can name the view: atlas.html#graph
  var asked = location.hash.slice(1);
  showView(["graph", "params", "findings"].indexOf(asked) >= 0 ? asked : (IX.findings.length ? "findings" : "graph"));
}
