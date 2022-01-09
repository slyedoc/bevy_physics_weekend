# Bevy Physics Weekend

A Native Bevy physics engine.

This is based on the 3 book series from [Game Physics in One Weekend](https://gamephysicsweekend.github.io/) (You can get all 3 for like $9 for kindle)

## Why

I have been playing with bevy for a while now and the only real option you have for physics is [Rapier](https://github.com/dimforge/rapier), either though [bevy_rapier](https://github.com/dimforge/bevy_rapier) or [heron](https://github.com/jcornaz/heron).  While rapier is truly amazing, it's not made for bevy, it uses its own math lib [nalgebra](https://github.com/dimforge/nalgebra) while bevy uses [glam](https://github.com/bitshifter/glam-rs) which leads to needing to copy transform data in and out every frame which just feels dirty.

Then there is the Bevy component issue with 0.6 that has consumed far too much developer time and still isn't resolved.

And lastly here is no interest in having a dev branch for bevy main, from Sebcrozet (he is rapier's main developer) view in makes sense, it's not worth tracking a moving target, but as bevy developer it's pretty painful.

## Goal

This will never be as feature rich as rapier, but hopefully it can be easier to use and understand.

I doubt I have the skills to get this to outperform rapier, in anything but simple use cases, but it should be fun to try.

## Notes

The 3 books have already been ported to bevy by Bitshifter [here](https://github.com/bitshifter/bevy-physics-weekend), the developer of the glam crate, finding that repo and knowing I can use it as ref gave me the confidence to even start this.
