// Closing the page menu by clicking away from it, and by pressing Escape.
//
// Everything else about the menu is the <details> element doing its own job -
// it opens, it closes, it takes the keyboard, and it does all of that with this
// file absent or failing to load. That is deliberate: one of the pages this
// runs on is the firmware updater, which has to work when the rest of the
// project does not, so nothing here may be load-bearing.

for (const menu of document.querySelectorAll('details.pages')) {
  // Not on click: a click that lands on the summary would close the menu the
  // same gesture is opening. pointerdown outside is unambiguous.
  document.addEventListener('pointerdown', ev => {
    if (menu.open && !menu.contains(ev.target)) menu.open = false;
  });

  document.addEventListener('keydown', ev => {
    if (ev.key !== 'Escape' || !menu.open) return;
    menu.open = false;
    // Back to the control that opened it, or the focus is left on nothing.
    menu.querySelector('summary')?.focus();
  });
}
