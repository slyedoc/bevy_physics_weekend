# Bevy Physics Weekend

A Native Bevy physics plugin.

This is based on the 3 book series [Game Physics in One Weekend](https://gamephysicsweekend.github.io/) which I highly recommend so far if physics engine interest you.  You can get all 3 for $9 on Amazon Kindle.

The 3 books have already been ported to bevy by Bitshifter [here](https://github.com/bitshifter/bevy-physics-weekend), the developer of the glam crate, finding that repo and knowing I can use it as ref gave me the confidence to even start this.

## Status

Still working though book 1 at the moment.

## Why

I have been playing with bevy for a while now and the only real option you have for physics is [Rapier](https://github.com/dimforge/rapier), either though [bevy_rapier](https://github.com/dimforge/bevy_rapier) or [heron](https://github.com/jcornaz/heron).  While rapier is truly amazing, it's not made for bevy, it uses its own math lib [nalgebra](https://github.com/dimforge/nalgebra) while bevy uses [glam](https://github.com/bitshifter/glam-rs).  This leads to needing to load data in and out every frame since they don't share types.

Then there is the Bevy component [issue](https://github.com/bevyengine/bevy/issues/2966) that has consumed far too much developer time and still isn't resolved.  Yes, bevy needs support for external types, but I want the physics in my game engine to be a first class citizen.

And the last reason, though admittedly the weakest, here is no interest in having a dev branch on bevy_rapier for bevy main branch, from Sebcrozet (rapier's main developer) view I get it, it's not worth tracking a moving target, but as a bevy developer it's pretty painful when you want something on main and the release cycle is long.

> I have tried 3 times to hack together an update to bevy_rapier for bevy 0.6 and failed, the bevy physics discord channel if full of others that have as failed as well.

## Goal

The aims to provide a simple bevy physics plugin that's completely native to bevy.

This will never be as feature rich as rapier, but hopefully it can be easier to use and understand.

I doubt I have the skills to get this to outperform rapier in anything but simple use cases, but it should be fun to try.
