
# Bevy Physics Weekend

A Native Bevy physics plugin.

Most likely you should go use rapier, though I did wonder what a native bevy implementation would look like.

The aims to provide a simple bevy physics plugin that's completely native to bevy. The goal is to be faster than rapier, even if not a feature rich.

## Acknowledgements

- [Game Physics in One Weekend](https://gamephysicsweekend.github.io/)
  - This Engine is based on the 3 book series, you can  can get all 3 for $9 on Amazon Kindle.
- [Bitshifter](https://github.com/bitshifter)
  - The 3 books have already been ported to bevy by [Bitshifter](https://github.com/bitshifter) in a repo [here](https://github.com/bitshifter/bevy-physics-weekend)
  - Bitshifter is the developer of the [glam](https://github.com/bitshifter/glam-rs) crate, finding that repo and knowing I can use it as ref gave me the confidence to even start this.
- [bevy_rapier](https://github.com/dimforge/bevy_rapier)
  - learning rapier and how it operated

## Screenshots

![App Screenshot](docs/images/screencapture-5000-balls.gif)

# Hi, I'm Slyedoc! 👋

I have [been](https://github.com/slyedoc) playing with bevy for a while now and the only real option you have for physics is [Rapier](https://github.com/dimforge/rapier), either though [bevy_rapier](https://github.com/dimforge/bevy_rapier) or [heron](https://github.com/jcornaz/heron).  While rapier is truly amazing, it's not made for bevy, it uses its own math lib [nalgebra](https://github.com/dimforge/nalgebra) while bevy uses [glam](https://github.com/bitshifter/glam-rs).

This leads to needing to load data in and out every frame since rapier knows nothing about ECS or its glam types, and some pain in needing to learn both math lib's once you start using it more though both crates try to limit this.

## Run Locally

Follow the `Install OS dependencies` at [bevy book](https://bevyengine.org/learn/book/getting-started/setup/).

```bash
cargo run  --release --example stack
```

If issues check the [Trouble shooting](./docs/troubleshooting.md) section.
