import React from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import UserMenu from '@site/src/components/Auth/UserMenu';

export default function NavbarItemWrapper(props) {
  if (props.type === 'custom-userMenu') {
    return <UserMenu {...props} />;
  }
  return <NavbarItem {...props} />;
}
