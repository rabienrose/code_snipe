<?php
$file_root = 'uploads';
echo '<p>the format is:</br>pitch, yaw, roll, ax, ay, az</p>';
foreach (glob($file_root."/*.txt") as $filename) {
	echo '<A href="'.$filename.'">'.$filename.' ('.filesize($filename).' byte)</A></br>';
}
?>