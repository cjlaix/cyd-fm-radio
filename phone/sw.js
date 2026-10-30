// Caches the page so it still opens in the car without mobile data.
const CACHE = "cyd-radio-v1";
const FILES = ["./", "index.html", "manifest.json", "icon.svg"];

self.addEventListener("install", e => {
  e.waitUntil(caches.open(CACHE).then(c => c.addAll(FILES)));
  self.skipWaiting();
});

self.addEventListener("activate", e => {
  e.waitUntil(caches.keys().then(keys => Promise.all(keys.filter(k => k !== CACHE).map(k => caches.delete(k)))));
  self.clients.claim();
});

// Network first (so updates show up), cache when offline.
// "no-cache" makes the browser check GitHub every time instead of reusing
// its own 10-minute copy, so a new version shows on the next load.
self.addEventListener("fetch", e => {
  e.respondWith(
    fetch(e.request, { cache: "no-cache" })
      .then(r => { const copy = r.clone(); caches.open(CACHE).then(c => c.put(e.request, copy)); return r; })
      .catch(() => caches.match(e.request))
  );
});
