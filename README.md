# Bevy Physics Weekend

A Native Bevy physics plugin.

Most likely you should go use rapier, though I did wonder what a native bevy impl would look like.

This is based on the 3 book series [Game Physics in One Weekend](https://gamephysicsweekend.github.io/), can get all 3 for $9 on Amazon Kindle.

The 3 books have already been ported to bevy by Bitshifter [here](https://github.com/bitshifter/bevy-physics-weekend),
the developer of the glam crate, finding that repo and knowing I can use it as ref gave me the confidence to even start this.

## Status

Finished all 3 book, refactoring to use ECS.

## Why

I have been playing with bevy for a while now and the only real option you have for physics is [Rapier](https://github.com/dimforge/rapier), either though [bevy_rapier](https://github.com/dimforge/bevy_rapier) or [heron](https://github.com/jcornaz/heron).  While rapier is truly amazing, it's not made for bevy, it uses its own math lib [nalgebra](https://github.com/dimforge/nalgebra) while bevy uses [glam](https://github.com/bitshifter/glam-rs).  This leads to needing to load data in and out every frame since rapier knows nothing about ECS or its glam types, and some pain in needing to learn both math lib's once you start using it more though both crates try to limit this.

## Goal

Learn about modern physics engines.

The aims to provide a simple bevy physics plugin that's completely native to bevy.

This will most likely never be as feature rich as rapier or as fast as rapier.
