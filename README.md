# Bevy Physics Weekend

A Native Bevy physics plugin.

Most likely you should go use rapier, though I did wonder what a native bevy impl would look like.

This is based on the 3 book series [Game Physics in One Weekend](https://gamephysicsweekend.github.io/) which I have learned a lot from, can get all 3 for $9 on Amazon Kindle.

The 3 books have already been ported to bevy by Bitshifter [here](https://github.com/bitshifter/bevy-physics-weekend), the developer of the glam crate, finding that repo and knowing I can use it as ref gave me the confidence to even start this.

## Status

A few chapters into book 2.  
Add rapier example for compairson.

## Why

I have been playing with bevy for a while now and the only real option you have for physics is [Rapier](https://github.com/dimforge/rapier), either though [bevy_rapier](https://github.com/dimforge/bevy_rapier) or [heron](https://github.com/jcornaz/heron).  While rapier is truly amazing, it's not made for bevy, it uses its own math lib [nalgebra](https://github.com/dimforge/nalgebra) while bevy uses [glam](https://github.com/bitshifter/glam-rs).  This leads to needing to load data in and out every frame since they don't share types.

## Goal

Learn about physics engines.

The aims to provide a simple bevy physics plugin that's completely native to bevy.

This will never be as feature rich as rapier and likely slower.
