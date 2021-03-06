# **Blender MD5 exporter**

Blender plugin that exports id Tech 4 MD5 meshes and animations.

## Installation

Copy to your Blender "addons" folder. In Linux, this is ~/.config/blender/scripts/addons.

## Usage

I've left the original manual for the addon included. It has guidance on how to set up your mesh / animation for export. 


## Wait, I've seen this code before!

Yes, this is a fork of the Arx "End of Sun" exporter for Blender 2.66, originally found at nemyax's post [here](http://www.katsbits.com/smforum/index.php?topic=520.0). I wasn't able to find it on Github (other blender md5 exporters are of Katsbit's exporter, which consistently would not work for me); and out of the interest to preserve and update the project, I'm forking it here. Full credit for the vast majority of this code goes to the Arx team, not me.

## What's changed?

1. Batch exporter no longer needs a selection; it exports the first object in the scene.
2. All actions are exported as their own animation, no longer need track markers.
3. Actions are exported prefixed with the name of the file you're targeting.
4. Tabs, not spaces.

## Assumptions about *.blend files

This fork is specifically made with the intent for each *.blend file to contain only one object, one mesh, one armature. The original exporter made no such assumptions, but I find that these assumptions make the exporter much easier to use headlessly (which is my purpose).