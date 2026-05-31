import React from 'react';
import Navbar from '@theme-original/Navbar';
import BooksBar from '@site/src/components/BooksBar';

// Wrap the default navbar to render a secondary "Books" bar directly beneath it.
export default function NavbarWrapper(props) {
  return (
    <>
      <Navbar {...props} />
      <BooksBar />
    </>
  );
}
