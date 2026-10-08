# VOSTOK 3D Printer Official Configuration

[🇷🇺 Русский](README.md) | 🇬🇧 English

![](./pics/fast_tool_swaps.webp)

Official configuration for the VOSTOK 3D printer. It uses an [experimental Klipper fork](https://github.com/dmbutyugin/klipper/tree/generic-cartesian) that supports direct control of both toolheads. This makes it possible to purge the idle toolhead while the active one is still printing, cutting the tool change time down to ~0.5–0.7 seconds, and also slightly speeding up parking and other similar moves.

## Installation

The flashing and installation procedure is covered in detail in several articles on the project website:

- [💾 Firmware](https://k3d.tech/vostok/manual/firmware/);
- [🔪 Slicer settings](https://k3d.tech/vostok/manual/slicer_configuration/);
- [🔄 Fast tool change script](https://k3d.tech/vostok/manual/fast_tool_swaps/).

Note that the articles are written in Russian. A lot of information on how to install and further tune this configuration is also provided directly in the `printer.cfg`, `printer_base.cfg` and `electronics_*.cfg` files.

Electronics configurations for boards maintained by users themselves live in the [`user_configs/`](user_configs/README.md) folder.

## Contributing your own configuration

1. Clone the repository to your computer:
   ```bash
   git clone https://github.com/dmitry-sorkin/vostok_configuration.git
   cd vostok_configuration
   ```
2. Create a separate branch:
   ```bash
   git checkout -b add-<board>
   ```
3. Add the configuration file to the `user_configs/` folder. Requirements for the file:
   - Location — `user_configs/`;
   - Name — `electronics_*.cfg`, where `*` describes the mainboard and the switch boards in use;
   - At the top of the file — the electronics wiring diagram used by this configuration;
   - You may also state the author and any other relevant information at the top of the file.
4. Commit the changes and push the branch:
   ```bash
   git add user_configs/electronics_<board>.cfg
   git commit -m "Add configuration for <board>"
   git push origin add-<board>
   ```
5. On GitHub, press `Compare & pull request`, fill in the description and submit the PR for review.

## FAQ

Q: Can't I just use mainline Klipper?

A: You can. But then there is no way to purge the idle toolhead before the swap. As a result, after every swap you would have to print a purge tower, which raises the swap time from ~0.5s to ~5s.

---

Q: Is it worth switching to this configuration?

A: If your printer already runs some older configuration and you are happy with it — leave it alone. Upgrading only makes sense if you need the fast toolhead change feature.

---

Q: How do I report a bug or a problem?
A: Open an issue describing the problem in detail.
